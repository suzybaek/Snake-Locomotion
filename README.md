## Snake Locomotion Implementation
**Result Video** https://youtu.be/_4xVoCZRFak

첫 참고 논문 : [Modeling and simulation of complex dynamic musculoskeletal architectures | Nature Communications](https://www.nature.com/articles/s41467-019-12759-5)

20/08/20

- 환경 세팅 및 elastica 실행
- 개념 학습 (논문 정리)

20/08/27

* muscularsnake를 실행시켜 나온 inc파일과 pov파일들을 povray로 렌더링
* 이미지 렌더링 하는 방법에 대해 고민하고 렌더링코드와 결과파일들을 분석
* scenepovray.inc 파일이 생성되지 않아서 렌더링 되지 않는 것이라 추측하여 임의로 scenepovray.inc 파일을 생성하여 povray로 렌더링하는 방향으로 해결

20/09/03

* 시뮬레이션 속도를 real time에 가깝게 하는 방법 찾아야 함
* 코드분석을 진행해 Parallelize 할 수 있는 부분을 찾아 시뮬레이션 시간을 줄여보고자 함

20/09/10

* 논문의 slithering 파트를 대략적으로 살펴보았음
* 주요 수식들과 관련있어 보이는 부분을 muscularsnake.cpp 파일에서 찾을 수 있었음

20/09/17

* 어떤 부분이 오래 걸리는지 간단하게 print로 알아봄
* 그 결과 Polymer.cpp의 integrate파트와 Window stats가 1개의 pov파일을 제작하는 동안 여러 번 발생하였고, 이 부분에 대해 멀티 쓰레딩하여 시간을 줄이는 방법을 찾아야겠다 생각
* Integrate 파트의 경우, 코드를 살펴보면 MuscularSnake.cpp에서 Polymer.cpp의 simulate함수를 호출하며, Polymer.cpp line 177에서 PositionVerlet2nd.cpp의 integrate를 호출하는데,
  Integrate 함수 내에서 쓰이는 수식들을 논문의 25p Appendix C 부분에서 확인할 수 있습니다. 그리고 논문에서는 이 계산 과정(논문 방정식 3.8, 3.9)이 가장 계산적으로 오래 걸린다고 말하고 있기 때문에, 계산 시간을 줄일 수 있는 방법이 있을지 고민
* 이를 멀티쓰레딩을 이용해 해결하려 할 경우, Main.cpp의 setup함수와 MRAGEEnvironment.h의 tbb::task_scheduler_init 부분을 참고하여 수정하면 될 것 같음
* 결론은 한 개의 pov파일을 생성할 때 수행하는 여러 번의 integrate를 멀티 스레드로 동작하게끔 수정해보고, integrate 계산 자체의 시간을 줄일 수 있는 방법에 대해 고민해보려고 함

20/09/21

* realtime simulation의 용도로는 한계가 있음 파악
* realtime 가까운 속도가 되려면 몇십배 정도 속도가 향상되어야
* 계산 속도 상의 이점과 일반적인 다른 물체와의 contact force 계산을 위해 범용 물리엔진 중 softbody 기능을 제공하는 라이브러리들을 써보기
* DRL 코드와의 결합성을 위해 가능하면 python library
* dartpy와 pybullet
* 둘 다 softbody가 외부 접촉에 반응하여 변형하는 기능은 제공하지만, 별도의 힘을 가해서 (근육힘과 같이) softbody가 변형하도록 하는 기능은 없는 것 같음
* 근육으로 actuation을 포기하고 3 dof joint로 연결된 여러 개의 rigid body들(즉 척추뼈들) 간의 joint torque로 몸체를 움직이되, softbody의 contact을 통해 다양한 지형 지물을 상에서 이동 시도

20/10/08 ~ 20/10/22

* dartpy 라이브러리에 softbody를 구현한 부분을 찾는 것을 목표로 해 이것이 rigid body와 attach할 수 있는지 확인

* pybullet)

  뱀 예제는 일단 제공하는 shape(상자:GEOM_BOX)을 불러와, 여러 개의 body들을 만듦
  여러 개의 body를 만들 때(createMultiBody) link를 두고 만들지, 없이 만들지 선택할 수 있음
  link를 둔다면 어떤 joint로 연결할지도 정할 수 있었다.
  이렇게 우선 뱀의 몸통을 만들고, 예제에서는 간단히 sin wave로 움직임. 이 부분에 이전에 공부한 논문 내용의 이론을 적용시키면 될 것 같음.

20/11/05

* 새로운 가상환경을 만들고 pybullet을 실행시켜본 뒤 코드를 간단하게 분석

20/11/12

* pybullet으로 아래 사진과 같이 1개의 rigid body를 1개의 soft body로 감싸는 것 성공

20/11/19

* soft body를 불러오면 느려지는 문제
  원인은 soft body를 많이 load 해오는 것으로 파악
  또한 softbody의 사이즈를 줄일수록 빨라짐

20/11/26

* 시뮬레이션 속도 향상을 위해 object들 사이의 collision을 없애보려 노력
  setCollisionFilterGroupMask와 setCollisionFilterPair 함수를 이용하여 rigid사이의 collision과 rigid와 soft사이의 collision을 없애 보려 함( enableCollsion값 설정)
* softbody를 load할 때 collision과 관련된 인자를 모두 0으로 셋팅
* softbody를 load하는 과정에서 시간이 많이 소요되는 것 같아 load함수를 한 번만 호출하도록 수정
* 그 후 time함수를 이용하여 480 step simulation에 대한 시간 차이를 계산해보니 4.62초 정도로 realtime과 약 2배정도 차이가 남
* 기존의 "1 rigid당 1 soft" 에서, achor 함수를 적절히 이용해 "1 soft + multi rigid"식으로 하나의 소프트 오브젝트를 여러 개의 rigid 바디들에 붙이는 것으로 수정

20/12/24

* 로드가 계속 안 되는 것을 보아 obj 파일 자체에 문제가 있다고 판단, 여러 가지 시도로(저번에 말씀드린 온라인 obj파일 생성 사이트로 시도) obj 파일을 생성해보다가 blender로 obj 파일을 생성해보았음
* blender로 만든 obj 파일은 잘 로드가 되는 것으로 파악

20/12/31

* DRL 개념 공부
* state,  action, reward 어떻게 구성할지 논의

21/01/07

* ppo2 라이브러리 공부

21/01/21

* openai gym

21/02/11

* state : 뱀의 머리, 다음 장애물과의 거리
* reward : 뱀의 속도

21/02/18

* gym 환경 불러오기

21/03/04

* state : 뱀의 각 노드의 좌표, 뱀의 각 노드의 속도, 각 관절의 각도, 마찰력
  환경의 고도, 땅의 거칠기

  1. 환경의 고도, 땅의 거칠기를 수치화

  2. (1) 뱀은 위 수치를 그래프화하자면 값이 낮은 쪽으로 이동하려고 할 것이므로 각 노드가 지나간 길의 값을 더한 것의 음수값을 reward 값으로 계산
     (2) 특정 속도로 이동하게 할 경우 Threshhold를 주어진 속도에 맞게 설정해 그 밖을 벗어날 경우엔 reward값을 감소시키는 식으로 계산

  action으로는 뱀의 각 관절의 토크를 설정해주면 될 것 같다.

21/03/11

* 코드 구현

21/03/18

* state에서 마찰력 고려 X

* state : 뱀 몸통의 중앙을 root로 잡고, 뱀의 각 부분의 위치들과 속도

  reward : 우선 평면위에서 가능한 빠르게 이동하는 것을 목표로 하기 위해, 목표 속도에 부합하는 정도에 따라 reward를 주려고 함

  action : 각 관절별 target pose를 pybullet의 setJointMotorMultiDofArrays 함수를 이용해 변경하는 식으로 구현하려고 함

21/03/25

* 본격적인 구현 시작
* singularity 세팅

21/04/01

* (1) init

   파이불릿을 연결하고 뱀의 모양을 만드는 부분

  action을 관절별 target pose를 설정하여 action range를 -np.pi/2 ~ np.pi/2 로 설정

  

  (2) reset

   reset함수는 random한 state를 부여하는 함수. 모델의 학습과 test가 시작 될 때 들어갈 state를 설정하기 위해서 만들었음.

  현재 10개 뱀 node의 3차원 Position + 10개 node의 3차원 Velocity로 state를 설정해주었기에 이에 맞게 random하게 값을 부여

  (Position : 중앙 root node의 x,y 값을 -10~10 값 사이에서 랜덤하게 부여한 뒤(z는 반지름) root노드를 기준으로 앞쪽 노드와 뒤쪽노드의 값을 설정

  Velocity : 3차원 속도 값을 -20~20의 값 사이에서 랜덤하게 부여 )

  

  (3) step

   모델에서 선택된 action이 setJointMotorMultiDofArrays의 targetPosition에 들어간 뒤 next state를 얻어냅니다.

  next state의 첫 번째 node의 velocity가 threshold보다 크면 reward를 받게 설정

  하지만 현재 뱀이 옆으로 자꾸 굴러가는 문제가 발생하여 뱀의 action_space와 threshold를 조정해가며 학습을 진행중

  (4) render : stepSimulation을 하는 함수

21/04/09

* threshold와 step함수에서 reward 계산하는 방법을 바꿔가며 학습을 진행

21/05/16

* reward 세분화 하여 설정
  threshold 를 사용하여 주지 않고 값 자체에 따라 reward를 설정한 뒤 가중치로 각 reward를 조정
  아래와 같은 네 가지 reward를 설정
  * head 로컬 Orientation 좌표와 goal 벡터의 내적을 이용해 각도를 얻고, 이 각도의 반비례 값을 reward_1으로 썼습니다.
  * 중앙 노드의 velocity 벡터와 goal 벡터의 내적값의 역수를 reward_2로 썼습니다.
  * getJointsTorqueSum함수를 이용해 토크의 총합을 구한 뒤 역수값으로 reward_3를 설정해주었습니다.
  * 급작스러운 속도의 변화에 대한 제한을 주기 위해 action을 취하기 전 노드들의 속도와 action을 취한 후의 노드들의 변한 속도의 차이를 최소화하기 위해 역수값을 취해 reward_4를 설정해주었습니다.
* local position 과 local velocity 사용
* random goal로 다양한 goal에 대응할 수 있게 구현
  또한 pybullet의 함수를 사용하여 goal의 지점을 text와 상자로 표시해주었습니다.
* training loss plot과 validation reward plot 을 ppo2.py 파일에 추가함으로써 학습 양상을 보면서 수렴하는 지점에서 학습을 중단하고 모델을 저장하고자 함
* Snake 길이 2배로

