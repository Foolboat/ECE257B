# ECE257B
The project of ECE257B

The matlab code is the implementation of our paper which creats an algorithm for the coordinated balloon observation system. We use Device to Device communication (D2D) and relays to improve the throughput and resources utilization rate of this system.

Our paper implements the communication process of the balloon observation system which uses large number of in-situ balloon sensors to observe hurricanes. For more details, please see our paper on this project. Most importantly, our algoritm use convex model to get the distribution of each relays and the number and the index of relay layers to receive the data from balloon sensors and send the data to the weather forecasting centers.

The first of our code is the parameter setting, which is important to the communication system.

The second part is the algorithm demonstration. In this part:
1. We get the number of balloon sensors in each layer.
2. Total number of relay layers.
3. The index of realy layers.
4. Calculate SNR for relays.
5. Calculate data rate for relays.
6. The number of relays that connect to a certain relay.
7. Calculate the queuing delay.
8. Set values for constraints (9-layer system). Values for constraints (12-layer system). Values for constraints (15-layer system). Values for constraints (18-layer system).

In the third part, we use convex model to calaulate optimal values of the parameter for 9-layer, 12-layer, 15-layer, 18-layer respectively.

In next part, we calculate the power efficiency for the four cases and compare relay system with the case of no relays which includes:
1. Estimate average data rate for balloon sensors for four cases (no relay).
2. The corresponding power consumption for four cases (no relay).
3. Calculate the power efficiency (mw/bps) for the cases without relays.

In the end, we generate plots to illustrate the improvement of our algorithm using D2D communication and relay.
1. Plot of total data rate.
2. Plot of total power consumption.
3. Energy consumption for transmitting one bit of data.
4. CDF plot when layer number are 9 and 15.
