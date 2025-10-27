# 단일 채널을 활용한 1cm 이하 고정밀 UWB 구현 및 응용

> **UWB(초광대역) 통신의 단일 주파수 채널에서 위상 오차를 제거하여, 기존 DS-TWR 방식의 거리 측정 오차를 1cm 이하로 개선한 고정밀 실시간 측위 시스템(RTLS)**

[![Award](https://img.shields.io/badge/Award-전자파학회%20동상-orange)]()
[![UWB](https://img.shields.io/badge/UWB-DW3000-blue)](https://www.qvo.com/products/p/DW3000)
[![MCU](https://img.shields.io/badge/MCU-nRF52840-green)](https://www.nordicsemi.com/products/nrf52840)
[![Framework](https://img.shields.io/badge/Framework-ROS-lightgrey)](https://www.ros.org/)

---

## 1. 프로젝트 개요

- 표준 UWB(초광대역) DS-TWR(Double-Sided Two-Way Ranging) 기술은 송수신기 간 위상 오차로 인해 수 cm 수준의 거리 측정 오차를 보이며, 이는 고정밀 위치 추적이 필요한 AR, 스마트 홈 등의 응용 분야에서 한계점으로 작용합니다.

- 본 프로젝트는 이러한 한계를 극복하기 위해 **단일 주파수 채널** 환경에서 패킷 교환 시 발생하는 공통 위상 오차(Common Phase Offset)를 효과적으로 제거하는 알고리즘을 제안하고 구현했습니다.
- UWB 신호의 I/Q 데이터로부터 위상 정보를 복원하고 이를 거리 계산에 반영하여, 최종적으로 거리 측정 오차를 **1cm 이하**로 개선하는 데 성공했습니다.

- **논문**: "단일 채널을 활용한 1 cm 이하의 고정밀 UWB 구현 및 응용." *한국전자파학회논문지* 36.8 (2025): 749-756. (장윤석, 배윤수, 박효준, 정현우, 한수민, 장병준.)
  
---

## 2. 제안 방식 (Enhanced DS-TWR)

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

> **[DS-TWR vs NEW DS-TWR 거리값 비교]**

 <div align="center" ><img width="70%" alt="image" src="https://github.com/user-attachments/assets/b6697ba5-b0b3-4c0e-ab38-031cb1340058" /></div>

#### 3단계: 필터링을 통한 좌표 안정화
보정된 거리값들을 LSM(Least Squares Method) 알고리즘에 적용하여 2D 좌표를 계산한 후, **Median 필터**와 **Kalman 필터**를 순차적으로 적용하여 최종 좌표의 노이즈와 지터를 제거하고 안정성을 확보합니다.

> **[기존 DS-TWR vs NEW DS-TWR 좌표 비교]**

 <div align="center">
  <img width="70%" alt="image" src="https://github.com/user-attachments/assets/13706900-f04a-47fc-a95b-577981941ae6" /> 
  </div>



---

## 3. 시스템 구성 및 데모

#### 하드웨어 구성
<div align="center">

<table align="center" width="90%">
  <tr>
    <th align="center">역할</th>
    <th align="center">칩셋 / 프레임워크</th>
    <th align="center">설명</th>
  </tr>
  <tr>
    <td align="center"><strong>Anchor / Tag</strong></td>
    <td align="center">Nordic nRF52840 (MCU)</td>
    <td align="center">메인 컨트롤러, 알고리즘 연산</td>
  </tr>
  <tr>
    <td align="center"><strong>Anchor / Tag</strong></td>
    <td align="center">Qorvo DW3000 (UWB)</td>
    <td align="center">DS-TWR 통신, I/Q 데이터 수집</td>
  </tr>
  <tr>
    <td align="center"><strong>중앙 처리 장치</strong></td>
    <td align="center">ROS (Framework)</td>
    <td align="center">다중 앵커 데이터 취합, 최종 좌표 계산 및 시각화</td>
  </tr>
</table>

</div>

#### 데이터 흐름
> 다중 Anchor(4개)는 Tag와의 거리를 각각 측정한 후, 이 거리 정보를 ROS 토픽을 통해 중앙 처리 장치로 전송합니다. 중앙 처리 장치는 수집된 거리값들을 기반으로 Tag의 최종 좌표를 실시간으로 계산하고 시각화합니다.


 <div align="center"><img width="70%"  alt="image" src="https://github.com/user-attachments/assets/f568a4da-809f-4c35-942a-1b0e504bc34e" /></div>


---

## 4. 결과 및 향후 연구

### ✅ 주요 성과
* **측정 정확도 1cm 이하 달성**: 기존 DS-TWR의 **오차 범위(수 cm)를 1cm 이하로 개선**하여 고정밀 측위의 가능성을 입증했습니다.
* **좌표 안정성 확보**: 필터링 알고리즘을 통해 최종 위치 좌표의 분산을 크게 줄여 RTLS의 신뢰성을 향상시켰습니다.
* **실시간 시스템 구현**: ROS 기반으로 다중 Anchor로부터 데이터를 실시간 수집 및 처리하는 시스템을 성공적으로 구축했습니다.

### ⚠️ 향후 연구 방향
* DS-TWR에서 교환되는 메시지 수를 줄이면서도 위상 복원 및 거리 보정을 효율적으로 수행할 수 있는 경량화된 통신 프로토콜을 연구하여 전력 효율성과 확장성을 높일 계획입니다.

---

## 5. 참고 문헌
* J. Ma, F. Zhang, B. Jin, C. Su, S. Li, Z. Wang, and J. Ni, “Push the limit of highly accurate ranging on commercial UWB devices,” in *Proceedings of the ACM on Interactive, Mobile, Wearable and Ubiquitous Technologies*, New York, NY, May 2024, pp. 1-27.

---

## 6. 데모 영상

<p align="center">
  <a href="https://www.youtube.com/watch?v=zzRW9kg5rdE">
    <img src="https://img.youtube.com/vi/zzRW9kg5rdE/0.jpg" width="600" alt="Demo Video Thumbnail">
  </a>
</p>

---

## 7. 수상
🏆 **한국 전자파 학회 제 5회 대학생 창의설계 경진대회 동상 수상**
