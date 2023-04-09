# Pybullet

## 조작키

g : full-screen

s : switch light

w : wire-frame

v : vector

a : collision frame?

ctrl + click : move camera

click : apply external force

## setJointMotorControlMultiDof 함수

spherical joint 에 대해 Joint Motor를 통제하는 함수.

Parameter에 대한 설명은 아래와 같다.

- `bodyUniqueId` : joint motor를 세팅할 multi-body의 id
- `jointIndex` : 몇 번째 joint에 대해 세팅할 것인지
- `controlMode` : joint motor 통제 방법 ([https://granitedevices.com/wiki/Control_modes](https://granitedevices.com/wiki/Control_modes) 참고)
    1. `POSITION_CONTROL` : joint의 각도를 조절하여 모터를 통제 (ex: pi/3 만큼 joint 를 회전시킨다.)
        - 우리가 사용할 것은 이것.
    2. `VELOCITY_CONTROL` : joint의 속도(아마 각속도)를 조절하여 모터를 통제 (ex: joint를 20rpm 속도로 회전시킨다.)
    3. `TORQUE_CONTROL` : joint의 돌림힘(토크)를 조절하여 모터를 통제 (ex: joint에 10Nm의 힘을 가해 회전시킨다.)
- `targetPosition` : `POSITION_CONTROL` 방식으로 통제시 사용하는 파라미터.
    - [ float, float, float] 또는 [float] 형식으로, 각 축에 대해 설정된다.
- `force` : `targetPosition` 에 도달되기 위해 사용되는 maximum motor force
- `maxVelocity` : `POSITION_CONTROL`시 maximum 속력을 설정한다.

---

# snake_origin.py

pybullet 활용 예시이며, baby_snake.py의 base 파일.

## 변수명 설명

- `dt`
- `m_wavePeriod` : 주기
- `m_waveLength` : 파동 길이
- `m_waveAmplitude` : 파동 진폭
- `m_waveFront` : 파동이 주기의 어디 부분부터 시작하는가?

## 더 살펴볼 것

- `setJointMotorControlMultiDof` 함수를 이용한 학습에 성공한 후, `setJointMotorControlMultiDofArray` 함수를 이용해서 관절 별로 다른 motor 설정을 사용하도록 수정하여 학습해야 한다.
- `setJointMotorControlMultiDof` 함수에서 `maxVelocity` 파라미터를 설정 이용하기.