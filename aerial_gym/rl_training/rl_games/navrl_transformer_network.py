"""NavRL++-Target structured temporal Transformer policy.

Token layout (17): [CLS] + static geometry + 5 obstacle-history + 5 robot-history +
5 target-track-history. Raw RGB-D, raw point clouds, simulator semantics, and GT target state are
not part of the input.
"""

import torch
import torch.nn as nn

from rl_games.algos_torch.network_builder import NetworkBuilder

from aerial_gym.task.navrl_task.navrl_perception import (
    MAX_OBSTACLES,
    OBSTACLE_DIM,
    OBSTACLE_HISTORY,
    ROBOT_DIM,
    ROBOT_HISTORY,
    STATIC_DIM,
    STRUCTURED_OBS_DIM,
    TARGET_DIM,
    TARGET_HISTORY,
)


EMBED_DIM = 64
NUM_TOKENS = 17


class NavRLTransformerBuilder(NetworkBuilder):
    def __init__(self, **kwargs):
        NetworkBuilder.__init__(self)

    def load(self, params):
        self.params = params

    def build(self, name, **kwargs):
        return NavRLTransformerBuilder.Network(self.params, **kwargs)

    class Network(NetworkBuilder.BaseNetwork):
        def __init__(self, params, **kwargs):
            actions_num = kwargs.pop("actions_num")
            input_shape = kwargs.pop("input_shape")
            NetworkBuilder.BaseNetwork.__init__(self)
            if int(input_shape[0]) != STRUCTURED_OBS_DIM:
                raise ValueError(
                    "navrl_transformer expects structured obs dim %d, got %d"
                    % (STRUCTURED_OBS_DIM, input_shape[0])
                )

            self.static_encoder = nn.Sequential(
                nn.Conv2d(1, 4, kernel_size=3, padding=1),
                nn.ELU(),
                nn.Conv2d(4, 16, kernel_size=3, stride=(2, 1), padding=1),
                nn.ELU(),
                nn.Conv2d(16, 16, kernel_size=3, stride=(2, 2), padding=1),
                nn.ELU(),
                nn.Flatten(),
                nn.Linear(16 * 1 * 18, 128),
                nn.ELU(),
                nn.Linear(128, EMBED_DIM),
            )
            self.obstacle_project = nn.Sequential(
                nn.Linear(MAX_OBSTACLES * OBSTACLE_DIM, 128), nn.ELU(), nn.Linear(128, EMBED_DIM)
            )
            self.robot_project = nn.Sequential(
                nn.Linear(ROBOT_DIM, 128), nn.ELU(), nn.Linear(128, EMBED_DIM)
            )
            self.target_project = nn.Sequential(
                nn.Linear(TARGET_DIM, 128), nn.ELU(), nn.Linear(128, EMBED_DIM)
            )
            self.cls_token = nn.Parameter(torch.zeros(1, 1, EMBED_DIM))
            self.position_embedding = nn.Parameter(torch.zeros(1, NUM_TOKENS, EMBED_DIM))
            nn.init.normal_(self.cls_token, std=0.02)
            nn.init.normal_(self.position_embedding, std=0.02)

            layer = nn.TransformerEncoderLayer(
                d_model=EMBED_DIM,
                nhead=4,
                dim_feedforward=128,
                dropout=0.1,
                activation="relu",
            )
            self.transformer = nn.TransformerEncoder(layer, num_layers=4)
            self.final_norm = nn.LayerNorm(EMBED_DIM)
            self.actor = nn.Sequential(
                nn.Linear(EMBED_DIM, 256), nn.ELU(), nn.Linear(256, 256), nn.ELU()
            )
            self.critic = nn.Sequential(
                nn.Linear(EMBED_DIM, 256), nn.ELU(), nn.Linear(256, 256), nn.ELU()
            )
            self.mu = nn.Linear(256, actions_num)
            self.sigma = nn.Parameter(torch.zeros(actions_num), requires_grad=True)
            self.value = nn.Linear(256, 1)
            nn.init.orthogonal_(self.mu.weight, 0.01)
            nn.init.constant_(self.mu.bias, 0.0)

        def is_rnn(self):
            return False

        def forward(self, obs_dict):
            obs = obs_dict["obs"]
            batch = obs.shape[0]
            offset = 0
            static = obs[:, offset : offset + STATIC_DIM].view(batch, 1, 4, 36)
            offset += STATIC_DIM
            obstacle = obs[
                :, offset : offset + OBSTACLE_HISTORY * MAX_OBSTACLES * OBSTACLE_DIM
            ].view(batch, OBSTACLE_HISTORY, MAX_OBSTACLES * OBSTACLE_DIM)
            offset += OBSTACLE_HISTORY * MAX_OBSTACLES * OBSTACLE_DIM
            robot = obs[:, offset : offset + ROBOT_HISTORY * ROBOT_DIM].view(
                batch, ROBOT_HISTORY, ROBOT_DIM
            )
            offset += ROBOT_HISTORY * ROBOT_DIM
            target = obs[:, offset : offset + TARGET_HISTORY * TARGET_DIM].view(
                batch, TARGET_HISTORY, TARGET_DIM
            )

            tokens = torch.cat(
                [
                    self.cls_token.expand(batch, -1, -1),
                    self.static_encoder(static).unsqueeze(1),
                    self.obstacle_project(obstacle),
                    self.robot_project(robot),
                    self.target_project(target),
                ],
                dim=1,
            )
            if tokens.shape[1] != NUM_TOKENS:
                raise RuntimeError("Transformer token schema drift: %d" % tokens.shape[1])
            tokens = tokens + self.position_embedding
            encoded = self.transformer(tokens.transpose(0, 1)).transpose(0, 1)
            cls = self.final_norm(encoded[:, 0])
            mu = self.mu(self.actor(cls))
            value = self.value(self.critic(cls))
            return mu, mu * 0.0 + self.sigma, value, None

