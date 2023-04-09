Snake Locomotion Implementation

21/03/18 3pm

승수("editCaps", "editCaps2") 
* 'snake-v0' -> 'Snake-v0'
* 'snakeEnv' -> 'SnakeEnv' 수정
* 중간에 추가되어버린 디버그 string들 정리("editCaps", "editCaps2")
* 'gym_test', 'ppo2_test' 정상 작동 확인 (계산머신에서도 돌려봐야할듯)

21/03/18 5pm

승수("editSnake")

- snake.py의 render()에서 p.connect 및 load 중복 삭제("editSnake")

21/03/21 1pm

혜원("set state and action")

- snake.py의 init함수와 step함수 수정

21/03/27 1pm

혜원("change state and edit reset(),init()")

- snake.py의 init, reset함수 수정. 승수가 톡방에 올린 그림 참고.

21/03/27 7pm

혜원("edit step()")

- snake.py의 step 함수 수정. render() 수정 필요.

21/03/28 7pm

진("model training test")

- ppo2_test.py 수정 

(1) loading a model without environment error 

 model save && load 주석 처리(학습 잘 된 모델 아직 없어서 저장 할 필요 x)

(2) ppo2 학습 횟수에 따른 test reward 확인

21/04/04 5pm

진("edit target pos & action space")

- snake.py 수정

- 옆으로 굴러가는 문제 해결 -> action space를 y좌표에 대해서만 설정해주니 해결되었음(pybullet의 snake.py의 targetPos에 들어가는 값이 Y값만 들어가는 것을 확인 후 수정)

- pybullet의 snake.py처럼 sin 등 추가하면 더 자연스럽게 앞으로 나아갈지도(?)

21/04/04 6pm

진("edit step")

- snake.py step함수에 wave부분 추가

21/04/04 9pm

혜원("add resetSimulation")

- snake.py reset 함수에 월드 초기화 부분 추가

21/04/06 10pm

혜원("add softbody")

- snake.py loadSnakeBody 함수 추가 후 makeSnakeBody함수에 추가

21/04/06 11pm

진("edit threshold and ppo2")

- snake.py threshold 수정 및 ppo2_test.py 하이퍼 파라미터 수정

21/04/24 7pm

진("edit ppo2.py")

- loss and test reward plot 함수 추가

21/05/05 2am

진("edit snake.py")

- reward 수정

21/05/05 3am

승수("mid index")

- base position을 뱀 중앙으로 변경

21/05/05 3am

혜원("add snake model")

- snake 클래스 분리

21/05/05 6pm

혜원("add functions for ppo2")

- SnakeModel.py 파일에 필요한 함수들 추가

21/05/05 6pm

혜원("upload snake_tmp.py for testing SnakeMode")

- SnakeModel을 사용하도록 snake.py 수정한 파일.
- 아직 테스트 해보기 전

21/05/09 2pm

진("snake_tmp 에러 수정")

- 컴파일 에러 수정

21/05/09 3pm

혜원("fix bug -ing...")

- resetSimulation 부분 옮김

21/05/10 1am

혜원("fix bugs from SnakeModel & Snake_tmp.py")

- SnakeModel의 getRootWorldPositionAndVelocity 함수와 getLinksLocalPositionAndVelocity 함수 수정

21/05/11 2pm

혜원("modify the way to calculate local coord")

- SnakeModel의 getRootWorldPositionAndVelocity 함수가 world position, orientation, velocity 세 가지를 리턴하도록 수정한 후 함수 이름 getRootWorldStates로 변경
- SnakeModel의 getLinksLocalPositionAndVelocity에서 world position, velocity -> local position, velocity 계산 방법을 알맞게 수정


21/05/12 6pm

진("modify rewards")

- reward_1 & reward_2 수정

21/05/13 2am

혜원("correct reward formulas")

- reward_1 & reward_2 수정
- 역수 사용하면 NAN 에러 발생하여 임시적으로 음수로 리워드에 들어가도록 설정해놓음

21/05/13 9am

혜원("fix SnakeModel segment fault")

- SnakeModel에서 p.connect 위치 수정

21/05/13 11am

혜원("double the length of snake")

- snake 길이 20으로 수정
- 임시로 길이 10짜리 softbody를 연달아 붙임
- goal은 프린트 되도록 했으나 직접 포인트를 그릴 수 있는 것이 좋을 것 같다.

21/05/13 9pm

진("training and evaluation")

- reward 음수 값 -> loss가 억대로 커짐
- reward의 Nan을 막기 위해 분모에 아주 작은 값 더해줌
- reward 별 크기 차이를 맞추기 위해 reward_1의 십의 단위에 맞춰 각각의 reward에 가중치를 부여함
- loss plot이 매우 불안정 -> reward 및 하이퍼 파라미터 조정 필요 

21/05/15 5am

진("training tuning")

1. 목표 지점 표시
- p.addUserDebugText 사용
- goal지점이 하늘에 설정되어 있어서 뱀이 자꾸 날라감 -> z좌표는 0으로 고정

2. p확률 조정
- goal이 매우 자주 바뀌어서 좀 더 작은 확률로 정정
- 이 확률로 잘 가는 것이 확인이 되면 좀 더 random하게 해도 될 것 같음

3. reward 가중치 조절 
- 다른 reward들 보다 goal의 방향으로 나아가는 게 가장 중요하다고 생각했기 때문에 reward_1의 가중치를 올려줌

4. 학습 양상 acc 폴더 추가 

21/05/15 1pm

혜원("draw goal box w.r.t mid of snake")

1. SnakeModel에 goal을 위한 BoxNode 클래스 추가
    - goal 부분에 상자를 그리고, debugtext도 표시

2. snake_tmp.py에 resetGoal함수를 추가함
    - random으로 local goal을 생성(뱀의 mid 기준)
    - local goal을 world goal로 바꿔서 BoxNode를 이용해 그림.

3. snake_tmp.py의 step함수에서 goal을 reset할 때 남아있던 debug text를 제거

21/05/16 5pm

혜원("change reward1 to use head ori instead of velo")

21/05/18 2pm

혜원("add some rewards & change midOrientaion")

1. SnakeModel에서 midOrientation 계산하는 방법 수정
2. SnakeModel에서 local pos, vel 계산하는 방법 수정
3. reward 몇 개 수정 및 추가