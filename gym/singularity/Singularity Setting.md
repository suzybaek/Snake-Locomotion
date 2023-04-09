## Singularity 기본 세팅

우선 tensorflow-1.15.5-gpu docker로 빌드 (python 3.6.9, tensorflow 1.15.5)

```
$ singularity build --sandbox tensorflow docker://tensorflow/tensorflow:1.15.5
```

pybullet 설치

```
$ pip install pybullet
```

gym 설치

```
$ pip install gym
```

root 권한 얻기 (mpi를 설치하기 위함)

```
$ su -
```

필요한 요소들 설치

```
$ apt-get update && apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev
```

stable-baselines 설치

```
$ pip install stable-baselines[mpi]
```



## Converting containers from one format to another

```
$ sudo singularity build production.sif development/
```

'development/' 라는 sandbox container(dir형식)를 sif 이미지로 변환



## Vagrant의 공유 폴더

Vagrantfile 있는 폴더 = VM의 /vagrant

즉 sif 파일을 /vagrant 폴더로 옮기면 내 컴퓨터의 폴더에도 표시가 된다.

그 반대도 성립.



## pscp.exe

Windows10 <-> Linux 파일 전송

cmd 창에서 pscp.exe {option} {파일명} {ID}@{보낼 IP}:{경로}

```
> pscp.exe -P 8022 tensorflow.sif teamA@gatecgr.hanyang.ac.kr:/home/teamA
```

-P 는 포트 번호 추가하는 옵션 (default : 22)



이러면 gatecgr.hanyang.ac.kr에 접속해보면 파일이 전송되어 있을 것이다.



## scp

Linux <-> Linux 파일 전송

scp {option} {파일명} {ID}@{보낼 IP}:{경로}

```
> scp tensorflow.sif teamA@192.168.0.105:/home/teamA
```



## Build

pink에서 build.

```
> singularity build tensorflow tensorflow.sif
```

