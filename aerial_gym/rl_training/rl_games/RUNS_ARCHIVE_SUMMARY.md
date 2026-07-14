# 옛 학습 run 요약 (삭제 전 보존) — 2026-07-14

`runs/`의 옛 run들을 삭제하기 전에 핵심 정보를 남김. 현재 학습 `ppo_260714_1904_navrl`은 유지.
지표: cap=captured_rate, crash=crash_rate, to=timeout_rate (최종 epoch). reward는 스케일이 리워드 설계에 따라 달라 절대비교 불가.

| run | type | ep | exit | last/peak reward | navrl 최종 | 무엇 |
|---|---|---|---|---|---|---|
| ppo_260530_1059 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1112 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1419 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1604 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1921 | intercept(옛과제) | 1261 | interrupted | 2408.2/2906.1 |  |  |
| ppo_260530_2005 | intercept(옛과제) | 6500 | interrupted | 922.4/1537.9 |  |  |
| ppo_260531_1031 | intercept(옛과제) | 15000 | max_epochs | 617.9/841.8 |  |  |
| ppo_260601_1152 | intercept(옛과제) | 931 | interrupted | 825.7/1149.7 |  |  |
| ppo_260601_1228 | intercept(옛과제) | 4652 | interrupted | 808.6/1076.9 |  |  |
| ppo_260601_1718 | intercept(옛과제) | 8787 | interrupted | 277.7/455.3 |  |  |
| ppo_260709_2146_navrl | navrl | 1500 | max_epochs | 142.5/170.3 |  | 첫 navrl 학습(1500ep, 옛 env). 목표도달 0%·충돌 다수 — 인프라만 완성. |
| ppo_260710_1230_navrl | navrl | 273 | interrupted | 126.5/159.8 |  | 초기 bars env 스모크/설정. |
| ppo_260710_1559_navrl | navrl |  |  | / |  | 초기 bars env 실험. |
| ppo_260710_1608_navrl | navrl | 1500 | max_epochs | 339.2/349.9 |  | bars env 첫 본학습(512env, MLP). |
| ppo_260710_1709_navrl | navrl |  |  | / | cap 0.00 / crash 0.37 / to 0.00 | bars env 실험. |
| ppo_260710_1738_navrl | navrl | 62 | interrupted | 229.7/236.4 | cap 0.00 / crash 0.42 / to 0.06 | bars env 실험. |
| ppo_260710_1745_navrl | navrl | 6000 | max_epochs | 365.2/375.9 | cap 0.00 / crash 0.02 / to 0.82 | bars env 실험. |
| ppo_260710_2106_navrl | navrl | 6000 | max_epochs | 296.9/327.5 | cap 0.01 / crash 0.12 / to 0.87 | 6000ep CNN 네비게이션 run(캡처종료 前, '목표찍고 배회'). |
| ppo_260713_1950_navrl | navrl | 6000 | max_epochs | 327.3/338.1 | cap 0.07 / crash 0.10 / to 0.84 | 캡처종료 첫 run — LOITER 실패(captured 6.7%, timeout 84%). 안전항 수입이 배회를 보상. |
| ppo_260713_2210_navrl | navrl | 6000 | max_epochs | 88.7/100.0 | cap 0.86 / crash 0.14 / to 0.00 | 리워드 재설계(B1 재베이스+PBRS progress+B3) 성공 — captured 86%(peak 94%), timeout 0%. loiter 해결. |
| ppo_260714_0153_navrl | navrl | 5049 | interrupted | 80.8/98.0 | cap 0.86 / crash 0.14 / to 0.00 | crash튜닝 A: safety_weight 1.5 — crash 13.7%(무효, safety만으론 안됨). |
| ppo_260714_0346_navrl | navrl | 6000 | max_epochs | 89.5/100.1 | cap 0.85 / crash 0.15 / to 0.00 | crash튜닝 B: clearance 거리 페널티 1.5 — crash 13.9%(무효). |
| ppo_260714_0555_navrl | navrl | 6000 | max_epochs | 85.6/100.2 | cap 0.87 / crash 0.13 / to 0.00 | crash튜닝 C: clearance speed-gated 1.5 — crash 13.8%(무효). 결론: 옛 스폰이 막대 무관(8%만 관통). |
| ppo_260714_1853_navrl | navrl | 32 | interrupted | 13.2/15.1 | cap 0.03 / crash 0.97 / to 0.00 | 짧게 중단된 run(설정 확인). |
