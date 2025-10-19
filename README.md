# 단일 채널을 활용한 1cm 이하 고정밀 UWB 구현 및 응용

> **UWB(초광대역) 통신의 단일 주파수 채널에서 위상 오차를 제거하여, 기존 DS-TWR 방식의 거리 측정 오차를 1cm 이하로 개선한 고정밀 실시간 측위 시스템(RTLS)**

[![Award](https://img.shields.io/badge/Award-전자파학회%20동상-orange)]()
[![UWB](https://img.shields.io/badge/UWB-DW3000-blue)](https://www.qorvo.com/products/p/DW3000)
[![MCU](https://img.shields.io/badge/MCU-nRF52840-green)](https://www.nordicsemi.com/products/nrf52840)
[![Framework](https://img.shields.io/badge/Framework-ROS-lightgrey)](https://www.ros.org/)
[![Publication](https://img.shields.io/badge/Publication-KIEES-_337-43B_)](https://www.jkiees.org/archive/view_article?pid=jkiees-36-8-749)

---

## 1. 프로젝트 개요

표준 UWB(초광대역) DS-TWR(Double-Sided Two-Way Ranging) 기술은 송수신기 간 위상 오차로 인해 수 cm 수준의 거리 측정 오차를 보이며, 이는 고정밀 위치 추적이 필요한 AR, 스마트 홈 등의 응용 분야에서 한계점으로 작용합니다.

본 프로젝트는 이러한 한계를 극복하기 위해 **단일 주파수 채널** 환경에서 패킷 교환 시 발생하는 공통 위상 오차(Common Phase Offset)를 효과적으로 제거하는 알고리즘을 제안하고 구현했습니다. UWB 신호의 I/Q 데이터로부터 위상 정보를 복원하고 이를 거리 계산에 반영하여, 최종적으로 거리 측정 오차를 **1cm 이하**로 개선하는 데 성공했으며, 이를 통해 RTLS의 전반적인 정확도와 신뢰성을 획기적으로 향상시켰습니다.

본 연구의 성과는 한국전자파학회논문지에 게재되었습니다.

* **논문**: 장윤석, 배윤수, 박효준, 정현우, 한수민, and 장병준. "단일 채널을 활용한 1 cm 이하의 고정밀 UWB 구현 및 응용." *한국전자파학회논문지* 36.8 (2025): 749-756.

---

## 2. 제안 방식 (Phase-Compensated DS-TWR)

본 프로젝트는 다음과 같은 3단계 프로세스를 통해 고정밀 거리 측정을 구현합니다.

#### 1단계: 위상 복원 (Recovery Phase)
DS-TWR 통신 과정에서 수신된 I/Q 데이터를 기반으로 신호의 위상($\Phi$)을 복원합니다. 이 복원된 위상값은 두 장치 간의 정밀한 ToF(Time of Flight) 오차 정보를 포함하고 있습니다.

$$
\Phi_{rec}(t) = (\Phi_{Poll} + \Phi_{Resp}) - (\Phi_{Final} - \Phi_{Post fin}) = (-2\pi(2f_c)\tau_{los}) \mod 2\pi 
$$

#### 2단계: 위상 정보를 이용한 거리 보정
기존 거리 측정값($D_{twr}$)과 복원된 위상($\Phi_{rec}$)을 결합하여 새로운 거리($D_{new}$)를 계산합니다. 이 과정에서 파장의 모호성(Ambiguity) 문제를 해결하기 위해 정수배 파장($N$)을 함께 추정하여 보정의 정확도를 높입니다.

$$
\lambda_{new} = \frac{c}{2f_c} \quad , \quad N = \text{round}(\frac{D_{twr}}{\lambda_{new}}) \quad , \quad D_{new} = (\frac{\Phi_{rec}}{2\pi} + N)\lambda_{new}
$$

> **[여기에 `DS-TWR vs NEW DS-TWR 거리값 비교` 그래프 삽입]**
>
> *(포스터의 `Original DS-TWR Data`와 `Filtered NEW DS-TWR Data`가 함께 있는 비교 그래프를 삽입하여, 제안 방식을 통해 거리 측정값의 분산이 크게 줄어든 것을 시각적으로 보여줍니다.)*

#### 3단계: 필터링을 통한 좌표 안정화
보정된 거리값들을 LSM(Least Squares Method) 알고리즘에 적용하여 2D 좌표를 계산한 후, **Median 필터**와 **Kalman 필터**를 순차적으로 적용하여 최종 좌표의 노이즈와 지터를 제거하고 안정성을 확보합니다.

> **[여기에 `기존 DS-TWR vs NEW DS-TWR 좌표 비교` 플롯 삽입]**
>
> *(포스터의 `기존 DS-TWR 2D 좌표 플롯`과 `NEW DS-TWR 2D 좌표 플롯`을 나란히 배치하여, 좌표의 분산이 확연히 줄어든 결과를 보여줍니다.)*

---

## 3. 시스템 구성 및 데모

#### 하드웨어 구성
-   **MCU**: Nordic nRF52840
-   **UWB IC**: Qorvo DWS3000
-   **통신 프레임워크**: ROS (Robot Operating System)

#### 데이터 흐름
다중 Anchor(4개)는 Tag와의 거리를 각각 측정한 후, 이 거리 정보를 ROS 토픽을 통해 중앙 처리 장치로 전송합니다. 중앙 처리 장치는 수집된 거리값들을 기반으로 Tag의 최종 좌표를 실시간으로 계산하고 시각화합니다.

> **[여기에 `시스템 구성도` 및 `실제 데모 환경 사진` 삽입]**
>
> *(포스터 좌측 하단의 `HW 구성` 다이어그램과 우측의 `Demo 환경 구성 사진`을 삽입하여 시스템의 전체적인 구조와 실제 실험 환경을 보여주세요.)*

---

## 4. 결과 및 향후 연구

### ✅ 주요 성과
-   **측정 정확도 1cm 이하 달성**: 기존 DS-TWR의 **오차 범위(수 cm)를 1cm 이하로 개선**하여 고정밀 측위의 가능성을 입증했습니다.
-   **좌표 안정성 확보**: 필터링 알고리즘을 통해 최종 위치 좌표의 분산을 크게 줄여 RTLS의 신뢰성을 향상시켰습니다.
-   **실시간 시스템 구현**: ROS 기반으로 다중 Anchor로부터 데이터를 실시간 수집 및 처리하는 시스템을 성공적으로 구축했습니다.

### ⚠️ 향후 연구 방향
-   DS-TWR에서 교환되는 메시지 수를 줄이면서도 위상 복원 및 거리 보정을 효율적으로 수행할 수 있는 경량화된 통신 프로토콜을 연구하여 전력 효율성과 확장성을 높일 계획입니다.

---

## 5. 저자 정보

-   **저자**: 장윤석, 배윤수, 박효준, 정현우, 한수민
-   **교신저자**: 장병준
-   **소속**: 국민대학교 전자공학부