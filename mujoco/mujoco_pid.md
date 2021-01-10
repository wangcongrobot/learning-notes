# MuJoCo PID Control


[Cascade PID Controller Implementation (#554)](https://github.com/openai/mujoco-py/commit/fe8373df3a36c4fb8631b0f3289fa8ecce182e0b)

* introduce an inverse dynamics (ID) controller. We select between PID vs ID custom controllers based on the actuator_user field that is defined per actuator. `user="1"` would trigger activating an ID controller where the gainprm are interpreted as Kp, Td, ema_smoothing_term, respectively.

* renaming

* Clean

* Cast to int

* wip cascade

* Enabled PID and cascaded PI control

* refactored PIDs

* fix id

* add some comments

* some more formatting

* test tweaks and fix to apply gravity compensation pre-clamping

* updated cascade documentation

* rm unused error term, move effort limit definition closer to where its used

* remove unused memory allocation

* simplify des_vel definition

* Co-authored-by: rubendsa <ruben@openai.com>


[Add PID-PI loop functionality (#561)](https://github.com/openai/mujoco-py/commit/89a953c5baea15b03fade1365ddf10d206591f68)

- Adds PID-PI position/velocity loops.
- EMA filter is moved to the outer position loop in cascaded control.


[PID implementation inside mujoco-py (#462)](https://github.com/openai/mujoco-py/commit/83759c2e20df965dcf727aa79e590a0483aaea0f)

* PID implementation inside mujoco-py

* addressing the comments from Jerry and Arthur

* add unit tests for new PID control

* add one more unit test for backward compatibility

* addressing comments

