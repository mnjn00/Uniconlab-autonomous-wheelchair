# UNICON 상용화 차단 문제 시각 보고서

팀원이 같은 문제 위치와 릴리스 조건을 공유하기 위한 보고서입니다. 파란색은 실제 좌표, 실행 결과, 또는 현재 증거가 끊긴 경계만 표시합니다.

## 바로 보기

- 화면용 원본: [`index.html`](index.html)
- 팀 공유 PDF: [`../../../output/pdf/unicon-commercial-readiness-visual-report.pdf`](../../../output/pdf/unicon-commercial-readiness-visual-report.pdf)
- 긴 PNG: [`../../../output/png/unicon-commercial-readiness-visual-report.png`](../../../output/png/unicon-commercial-readiness-visual-report.png)
- 경로 판정 결과: [`assets/route-band-audit.json`](assets/route-band-audit.json)

저장소 루트에서 아래 명령을 실행하면 근거 파일 링크까지 함께 열립니다.

```bash
python3 -m http.server 8767 --bind 127.0.0.1
```

브라우저 주소:

```text
http://127.0.0.1:8767/decks/unicon-commercial-readiness/visual-report/index.html
```

## 경로 그림 다시 만들기

실행 맵이 연결된 Mac에서 저장소 루트를 기준으로 실행합니다.

```bash
uv run decks/unicon-commercial-readiness/visual-report/generate_route_visuals.py
```

생성 입력:

- `routes/20260727_new_route_safety_band.json`
- `routes/20260727_new_route_waypoints.json`
- `src/static_livox_localization/scripts/safety_band.py`
- `/Volumes/무제/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd`

## 해석 범위

경로 그림은 저장된 밴드와 기록 경로가 현재 `SafetyBand` 포함 판정에 실패하는 위치를 보여 줍니다. 실제 연석 침범이나 충돌 재현 결과는 아닙니다. 승객 탑승과 일반 캠퍼스 주행을 열기 전 경로 번들을 다시 만들고, Gate 0부터 현장 근거를 닫아야 한다는 차단 자료입니다.
