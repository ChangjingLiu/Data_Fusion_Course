# Data Fusion
> This are code repo for the Data Fusion course in SJTU.
> Live demo [_here_](https://www.example.com). <!-- If you have the project hosted somewhere, include the link here. -->

## Table of Contents
* [General Info](#general-information)
* [Optimal Estimation](#optimal-estimation)
* [Wiener Filter](#wiener-filter)
* [Kalman Filter](#kalman-filter)
* [Setup](#setup)
* [Usage](#usage)
* [Project Status](#project-status)
* [Room for Improvement](#room-for-improvement)
* [Acknowledgements](#acknowledgements)
* [Contact](#contact)
<!-- * [License](#license) -->


## Optimal Estimation
### Problem description
Suppose a voltage is a random variable $X$ with normal distribution, the mean value is $5$, and the variance is $0.1$; The random variable x is measured $20$ times by two instruments, and the measurement error of the two instruments is assumed to be a normally distributed random variable with a mean value of $0$ and a variance of $0.1$ and $0.4$ respectively. Caculate the least square estimation (LSE), weighted least square estimation (WLS) and linear minimum variance estimation (LMMSE) of $X$, and calculate the mean square error of the corresponding estimation. Let the measurement equation be $Z=HZ+V$.

### Usage
To handle the problem, run the following file:

`1/code_1/main123.m`
<!--
### Result
|  Method   | $\hat{X}$ estimation  |MSE|
| :-----| :----: | :----:|
| LSE  | 5.0615 |0.0063|
| WLS  | 5.0292 |0.0040|
| LMMSE  | 5.0281 |0.0038|
-->



<!-- You don't have to answer all the questions - just the ones relevant to your project. -->


## Wiener Filter
### problem description
Let $y (n) =x (n) +v (n)$, where $x(n)=10sin(\frac{\pi n}{128}+\frac{\pi}{3})$,$v(n)$
is white noise with variance of $1.25$. Design FIR and IIR Wiener filter to estimate the signal $x (n)$.
### Usage
To handle the problem, run the following file:

`1/code_1/main.m`

<!--
### Result
![wiener_filter](./1/code_2/img/wiener_filter.png)
-->

## Kalman Filter

### Basic Kalman Filter
`1/code_3/kalman.m`
### Constant Gain Kalman Filter
`1/code_3/kalman_constant_gain.m`
### Square root Kalman Filter
`1/code_3/kalman_sqrt.m`
### Forgetting Factor Kalman Filter
`1/code_3/kalman_forgetting_factor.m`
### Adaptive Kalman Filter
`1/code_3/kalman_adaptive.m`
### Limited K Reduction Kalman Filter
`1/code_3/kalman_restain_K.m`

### Extended Kalman Filter
`2/code_0/EKF.m`
### Unscented Kalman Filter
`2/code_0/UKF.m`
### Particle Filter
`2/code_0/PF.m`

### Federated Kalman Filter
`2/code_1/federated_filter.m`
### Decentralized Kalman filter
`2/code_1/center_federated_filter.m`

## Fuzzy Control
### Basic method
`4/code/TS_model.m`
### T-S method
`4/code/TS_model.m`


## Contact
changjingliu@sjtu.edu.cn


<!-- Optional -->
<!-- ## License -->
<!-- This project is open source and available under the [... License](). -->

<!-- You don't have to include all sections - just the one's relevant to your project -->
