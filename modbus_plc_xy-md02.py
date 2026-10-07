import time
import logging
from pymodbus.client import ModbusSerialClient

# 통신 패킷 디버깅 로그
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

PORT = "COM4"
PLC_SLAVE_ID = 2      # PLC 국번
SENSOR_SLAVE_ID = 1   # XY-MD002 센서 국번 (공장출시 국번)

# 1. 시리얼 클라이언트 초기화
client = ModbusSerialClient(
    port=PORT, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=2
)

# 입김 수준의 미세 변화를 감지하기 위한 기준점 (최초 측정값 기준으로 자동 보정)
base_temp = None
base_humi = None

def get_bit_array(current_val, base_val, step):
    """
    현재 값과 기준값의 차이를 계산하여 8개 비트 배열(True/False)로 변환하는 함수
    예: 기준보다 높으면 높을수록 True 개수가 많아짐 (레벨 메터 방식)
    """
    diff = current_val - base_val
    if diff < 0:
        diff = 0
    
    # 초과한 단계 수 계산 (최대 8단계)
    active_bits = min(int(diff / step), 8)
    
    # 8개 비트 배열 생성 (예: active_bits가 3이면 [True, True, True, False, False, False, False, False])
    return [True] * active_bits + [False] * (8 - active_bits)

try:
    if not client.connect():
        print("시리얼 포트 연결 실패")
        raise SystemExit

    print("포트 연결 성공. 온습도 감지 루프를 시작합니다. (종료: Ctrl+C)")
    print("-" * 60)

    while True:
        # 2. XY-MD002 센서로부터 데이터 읽기 (Input Register 1번지부터 2개 읽기, 함수코드 0x04)
        sensor_reply = client.read_input_registers(address=1, count=2, slave=SENSOR_SLAVE_ID)
        
        if sensor_reply is None or sensor_reply.isError():
            print(f"[경고] 센서 응답 오류: {sensor_reply}")
            time.sleep(1)
            continue
            
        # 데이터 원본 가져오기 및 10분할 변환
        raw_temp = sensor_reply.registers[0]
        raw_humi = sensor_reply.registers[1]
        
        # 온도 음수 처리 적용
        cur_temp = raw_temp / 10.0 if raw_temp < 32768 else (raw_temp - 65536) / 10.0
        cur_humi = raw_humi / 10.0
        
        # 최초 실행 시 현재 상온/상습을 기준점(Baseline)으로 셋팅
        if base_temp is None:
            base_temp = cur_temp - 0.2  # 미세한 입김 초기 반응성을 위해 기준을 살짝 낮춤
            base_humi = cur_humi - 1.0
            # 💡 오타 수정 완료: :.1s -> :.1f
            print(f"✨ 초기 기준점 설정 완료 -> 기준 온도: {base_temp:.1f}℃, 기준 습도: {base_humi:.1f}%")

        # 3. 입김 감지를 위한 미세 스케일링 설정
        # - 온도: 기준값보다 0.2도 오를 때마다 비트 1개씩 추가 불 켜짐 (최대 +1.6도 감지)
        # - 습도: 기준값보다 3.0% 오를 때마다 비트 1개씩 추가 불 켜짐 (최대 +24% 감지)
        temp_bits = get_bit_array(cur_temp, base_temp, step=0.2)
        humi_bits = get_bit_array(cur_humi, base_humi, step=3.0)
        
        # PLC에 한 번에 전송할 전체 16개 비트 결합 (온도 8개 + 습도 8개)
        # M30~M37: 온도 비트, M38~M45: 습도 비트
        total_coils = temp_bits + humi_bits

        # 4. PLC M영역에 데이터 쓰기 (M30 시작번지 = 파이썬 address 0)
        plc_reply = client.write_coils(address=0, values=total_coils, slave=PLC_SLAVE_ID)
        
        if plc_reply is None or plc_reply.isError():
            print(f"[경고] PLC 쓰기 오류: {plc_reply}")
        else:
            # 터미널에 시각적으로 진행 상황 출력
            t_bar = "🔥" * temp_bits.count(True) + "  " * (8 - temp_bits.count(True))
            h_bar = "💧" * humi_bits.count(True) + "  " * (8 - humi_bits.count(True))
            print(f"[측정] 온도: {cur_temp:.1f}℃ [{t_bar}] | 습도: {cur_humi:.1f}% [{h_bar}]")

        # 0.5초 주기로 센서 수집 및 PLC 동기화 반복
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n사용자에 의해 프로그램이 종료되었습니다.")

except Exception as e:
    print(f"\n실행 중 예외 에러 발생: {repr(e)}")

finally:
    client.close()
    print("포트 연결이 안전하게 종료되었습니다.")
