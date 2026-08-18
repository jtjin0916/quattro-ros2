# Legacy reference

이 폴더는 리팩터링 전 원본 동작을 비교/분석하기 위한 기록용 코드다.

- `mit_publisher_ros2_legacy.py`: 기존 Raspberry Pi SocketCAN 기반 GIM6010 MIT driver 원본
- `mpc_controller_failed.py`: 실패 실험으로 분류한 MPC 코드 원본

새 `setup.py`의 console_scripts에는 등록되어 있지 않으므로 정상 launch에서는 실행되지 않는다.
