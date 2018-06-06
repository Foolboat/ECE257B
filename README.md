# ECE257B
The project of ECE257B

The matlab code is the implementation of our paper which creats an algorithm for the coordinated balloon observation system. We use Device to Device communication (D2D) and relays to improve the throughput and resources utilization rate of this system.

Our paper implements the communication process of the balloon observation system which uses large number of in-situ balloon sensors to observe hurricanes. For more details, please see our paper on this project. Most importantly, our algoritm uses convex model to get the distribution of each relays as well as the number and the index of relay layers. The uploaded data from balloon sensors is then utilized by the BS for weather forecasting. 

The first part of our code is parameter setting, which configures initial conditions for the system.

The second part is the algorithm implementation. In this section, we express:
1. Total number of relay layers.
2. The index of realy layers.
3. Calculate SNR for relays.
4. Calculate data rate for relays.
5. The number of relays that connect to a certain relay.
6. Calculate the queuing delay.
7. Set values for constraints (9-layer, 12-layer, 15-layer and 18 layer systems).

In the third part, we use convex optimization model to calaulate optimal values of the variable (number of relays in each relay layer) for 9-layer, 12-layer, 15-layer, 18-layer system respectively.

In addition, we also calculate the power utilization efficiency for the four cases and compare the result of relay system with the case of no relays, which includes:
1. Estimate average data rate for balloon sensors for four cases (no relay).
2. The corresponding power consumption for four cases (no relay).
3. Calculate the power efficiency (mw/bps) for the cases without relays.

In the end, we generate plots to illustrate the performance of our algorithm using cooperative transmissions and relays.
1. Plot the total data rate of the system.
2. Plot of total power consumption.
3. Energy consumption for transmitting one bit of data.
4. CDF plot when layer numbers are 9 and 15.
