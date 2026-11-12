// MOTAR RGB-D Gazebo bridge plugin. Separate from the byte-pinned historical S4 bridge.
//
// Gives the MOTAR SITL bridge Gazebo's own sensor output in place of the bridge's ray casts, and moves the target:
//   - keeps the latest frame of the vehicle's gpu_ray LiDAR (polled in the world-update thread), RGB and depth cameras
//     (copied in the rendering thread on each new frame), each stamped with the rendering scene's simulation time;
//   - applies the target pose the bridge sends (a kinematic, collision-free model), at the next world update;
//   - serves both over one localhost TCP connection (little-endian):
//       'T' + 3 float64 (x, y, z world)  -> reply float64 sim time at receipt
//       'R'                              -> reply the frame packet (see SendFrames)
// Nothing here changes physics: the target has no collision element and the sensors are rendering-only.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo {

class MotarRgbdBridge : public WorldPlugin {
 public:
  ~MotarRgbdBridge() override {
    stop_ = true;
    if (listen_fd_ >= 0) { shutdown(listen_fd_, SHUT_RDWR); close(listen_fd_); }
    if (client_fd_ >= 0) { shutdown(client_fd_, SHUT_RDWR); }
    if (server_.joinable()) server_.join();
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    port_ = sdf->HasElement("port") ? sdf->Get<int>("port") : 4590;
    target_ = sdf->HasElement("target") ? sdf->Get<std::string>("target") : "motar_target";
    // The camera-only rig has no LiDAR element or device. Its packet retains
    // zero dimensions, an empty payload and the -1 timestamp sentinel.
    lidar_name_ = sdf->HasElement("lidar") ? sdf->Get<std::string>("lidar") : "";
    rgb_name_ = sdf->Get<std::string>("rgb");
    depth_name_ = sdf->Get<std::string>("depth");
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(port_));
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 || listen(listen_fd_, 1) != 0) {
      gzerr << "[motar_rgbd] cannot listen on 127.0.0.1:" << port_ << "\n";
      return;
    }
    server_ = std::thread(&MotarRgbdBridge::Serve, this);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&MotarRgbdBridge::OnUpdate, this));
    gzmsg << "[motar_rgbd] listening on 127.0.0.1:" << port_ << "\n";
  }

 private:
  void Connect() {
    if (!lidar_name_.empty() && !lidar_)
      lidar_ = std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensors::get_sensor(lidar_name_));
    if (!rgb_) rgb_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensors::get_sensor(rgb_name_));
    if (!depth_) depth_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensors::get_sensor(depth_name_));
    // RGB: copied in the rendering thread under the frame lock, so a frame is never read while Gazebo writes it
    // (S3c/S4 review r1); stamped with the rendering scene's simulation time, which can lag physics (review r2).
    if (rgb_ && !rgb_conn_ && rgb_->Camera()) {
      rgb_conn_ = rgb_->Camera()->ConnectNewImageFrame(
          [this](const unsigned char* d, unsigned int w, unsigned int h, unsigned int, const std::string&) {
            std::lock_guard<std::mutex> g(frame_mu_);
            rgb_w_ = w;
            rgb_h_ = h;
            image_.assign(d, d + static_cast<size_t>(w) * h * 3);
            rgb_t_ = rgb_->Camera()->GetScene()->SimTime().Double();   // the rendered scene's time (r2 s1)
          });
    }
    // Gazebo fills the depth buffer only for a subscriber of the depth camera's new-frame event (DepthData() stays
    // null otherwise, smoke 2026-11-06): copy each frame in the rendering thread, stamped with the scene's sim time.
    if (depth_ && !depth_conn_ && depth_->DepthCamera()) {
      depth_conn_ = depth_->DepthCamera()->ConnectNewDepthFrame(
          [this](const float* d, unsigned int w, unsigned int h, unsigned int, const std::string&) {
            std::lock_guard<std::mutex> g(frame_mu_);
            depth_w_ = w;
            depth_h_ = h;
            depthv_.assign(d, d + static_cast<size_t>(w) * h);
            depth_t_ = depth_->DepthCamera()->GetScene()->SimTime().Double();   // the rendered scene's time
          });
    }
  }

  void OnUpdate() {
    Connect();
    {
      std::lock_guard<std::mutex> g(pose_mu_);
      if (have_pose_) {
        physics::ModelPtr m = world_->ModelByName(target_);
        if (m) {
          m->SetWorldPose(ignition::math::Pose3d(px_, py_, pz_, 0, 0, 0));
          applied_t_ = world_->SimTime().Double();
          applied_count_++;
        }
        have_pose_ = false;
      }
    }
    std::lock_guard<std::mutex> g(frame_mu_);
    if (lidar_) {
      double t = lidar_->LastMeasurementTime().Double();
      if (t > lidar_t_) {
        std::vector<double> r;
        lidar_->Ranges(r);
        ranges_.assign(r.begin(), r.end());
        lidar_t_ = t;
        lidar_h_ = lidar_->VerticalRangeCount();
        lidar_w_ = lidar_->RangeCount();
      }
    }
  }

  static bool ReadAll(int fd, void* buf, size_t n) {
    char* p = static_cast<char*>(buf);
    while (n) {
      ssize_t k = recv(fd, p, n, 0);
      if (k <= 0) return false;
      p += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }

  static bool WriteAll(int fd, const void* buf, size_t n) {
    const char* p = static_cast<const char*>(buf);
    while (n) {
      ssize_t k = send(fd, p, n, MSG_NOSIGNAL);
      if (k <= 0) return false;
      p += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }

  // Packet: magic 'MS4F', uint32 version 1; float64 sim_now, lidar_t, rgb_t, depth_t, target_applied_t;
  // uint32 target_applied_count, lidar_h, lidar_w, rgb_w, rgb_h, depth_w, depth_h;
  // float64[lidar_h*lidar_w] ranges (Gazebo order), uint8[rgb_h*rgb_w*3] RGB, float32[depth_h*depth_w] depth.
  bool SendFrames(int fd) {
    std::vector<char> out;
    auto put = [&out](const void* p, size_t n) { const char* c = static_cast<const char*>(p); out.insert(out.end(), c, c + n); };
    double now = world_->SimTime().Double();
    double at;
    uint32_t count;
    {
      std::lock_guard<std::mutex> g(pose_mu_);
      at = applied_t_;
      count = applied_count_;
    }
    std::lock_guard<std::mutex> g(frame_mu_);
    const char magic[4] = {'M', 'S', '4', 'F'};
    uint32_t version = 1;
    put(magic, 4);
    put(&version, 4);
    double ts[5] = {now, lidar_t_, rgb_t_, depth_t_, at};
    put(ts, sizeof(ts));
    uint32_t dims[7] = {count, lidar_h_, lidar_w_, rgb_w_, rgb_h_, depth_w_, depth_h_};
    put(dims, sizeof(dims));
    put(ranges_.data(), ranges_.size() * sizeof(double));
    put(image_.data(), image_.size());
    put(depthv_.data(), depthv_.size() * sizeof(float));
    return WriteAll(fd, out.data(), out.size());
  }

  void Serve() {
    while (!stop_) {
      int fd = accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) { if (stop_) return; continue; }
      int one = 1;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      client_fd_ = fd;
      while (!stop_) {
        char cmd;
        if (!ReadAll(fd, &cmd, 1)) break;
        if (cmd == 'T') {
          double p[3];
          if (!ReadAll(fd, p, sizeof(p))) break;
          {
            std::lock_guard<std::mutex> g(pose_mu_);
            px_ = p[0]; py_ = p[1]; pz_ = p[2];
            have_pose_ = true;
          }
          double now = world_->SimTime().Double();
          if (!WriteAll(fd, &now, sizeof(now))) break;
        } else if (cmd == 'R') {
          if (!SendFrames(fd)) break;
        } else {
          break;
        }
      }
      close(fd);
      client_fd_ = -1;
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_, depth_conn_, rgb_conn_;
  std::thread server_;
  std::atomic<bool> stop_{false};
  int listen_fd_ = -1;
  std::atomic<int> client_fd_{-1};
  int port_ = 4590;
  std::string target_, lidar_name_, rgb_name_, depth_name_;
  sensors::GpuRaySensorPtr lidar_;
  sensors::CameraSensorPtr rgb_;
  sensors::DepthCameraSensorPtr depth_;
  std::mutex pose_mu_, frame_mu_;
  bool have_pose_ = false;
  double px_ = 0, py_ = 0, pz_ = -10, applied_t_ = -1;
  uint32_t applied_count_ = 0;
  double lidar_t_ = -1, rgb_t_ = -1, depth_t_ = -1;
  uint32_t lidar_h_ = 0, lidar_w_ = 0, rgb_w_ = 0, rgb_h_ = 0, depth_w_ = 0, depth_h_ = 0;
  std::vector<double> ranges_;
  std::vector<unsigned char> image_;
  std::vector<float> depthv_;
};

GZ_REGISTER_WORLD_PLUGIN(MotarRgbdBridge)

}  // namespace gazebo
