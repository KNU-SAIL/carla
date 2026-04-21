# UVARC_CARLA 0.10.0 

## 1. 수정사항 요약

### 기능 추가

- `870cf8249` - `[Add] 데이터 로깅 예제`
- `337642de0` - `[Add] Advanced IMU 모드`
- `cd82b2d53` - `[Add] 가속도계 Turn-On 바이어스`
- `9cfaba749` - `[Add] 카메라 Grayscale 모드`

### 버그 수정 및 유지보수

- `629d1caca` - `[Fix] 가속도계 초기 스파이크 방지`
- `84291c0b8` - `[Fix] 카메라 Intrinsic 초점거리 계산 오탈자`
- `f539a5c14` - `[Fix] libfoonathan_memory 버전 불일치로 인한 stage 실패 해결`
- `fca933aaf` - `[Fix] Town15 맵 cook 시 SIGSEGV 크래시 해결`
- `5b5ccbfd0` - `[Fix] UE5 API 변경에 따른 함수명 갱신`

## 2. 설치 및 패키징 절차

아래 절차는 Linux 기준.

### 2-1. Git clone

```bash
git clone -b uvarc/main https://github.com/KNU-SAIL/carla.git
cd carla
```


### 2-2. `./CarlaSetup.sh --interactive` 실행

루트 디렉터리에서 아래 명령을 실행.

```bash
./CarlaSetup.sh --interactive
```

이 단계에서 다음 작업이 진행됩니다.

- 필수 패키지 설치
- Unreal Engine 관련 의존성 준비
- CARLA 빌드 환경 구성

실행 중 Unreal Engine 5.5 레포지터리 접근을 위해서 GitHub 인증이 필요. [가이드 참조](https://www.unrealengine.com/en-US/ue-on-github).

### 2-3. `autoExposureApplyPhysicalCameraExposure` 값을 `true`로 변경
- CARLA 0.10.0 배포 테스트 중, 카메라 영상에서 Traffic sign을 제외한 대부분의 객체에 조명이 정상 적용되지 않아 검게 보이는 현상이 확인됐다.
- 원인은 PostProcess 설정값인 autoExposureApplyPhysicalCameraExposure 때문이며, 배포 테스트 기준으로는 값을 false에서 true로 변경해야 정상적으로 표시된다.
  다만 수정 대상 파일은 현재 GitHub의 CARLA 소스 레포에서 직접 관리되는 파일이 아니라, setup 과정에서 별도로 내려받는 CARLA content 에셋에 포함된다.
  따라서, 빌드 전 수동으로 수정해야한다.

다음 파일을 엽니다.

- `Unreal/CarlaUnreal/Content/Carla/Config/PostProcess/Default.json`


아래 항목을 찾습니다.

```json
"autoExposureApplyPhysicalCameraExposure": false
```

다음과 같이 수정합니다.

```json
"autoExposureApplyPhysicalCameraExposure": true
```


### 2-4. 패키지 빌드

마지막으로 아래 명령을 실행합니다.

```bash
cmake --build Build --target package
```
