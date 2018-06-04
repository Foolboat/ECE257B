%  The demonstration of the project algorithm

%% Related Parameters

%Transmitting Power of Relay,120mW
P_R = 120 * 10^(-3); 
%Transmitting Power of balloon sensor, 40mW
P_T = 40 * 10^(-3);
%Uplink Bandwidth, 5 MHz
B = 20 * 10^3;
%Number of the system layer
L = [9, 12, 15, 18];
%Number of balloon sensors
N = [216, 360, 540, 756];
%Lower limit of the transmission rate, 4 bit/s
ST_0 = 4;
%Delay requirement of balloons, <30s
D_0 = 30 * 10^3;
%Transmission delay, 4ms
E = 4;
%Limited number of T can be connected by R.
n = 10;
%System radius range (m).
d_min = 50 * 10^3;
d_max = 100 * 10^3;
%SNR (dB) for balloon sensor
SNR_T = 15;
SNR_m = 10;


%% Algorithm demonstration

%The number of balloon sensors in each layer
N_i1 = zeros(1, 9);
for i = 1:9
    N_i1(i) = 8 + (i - 1) * 4;
end

N_i2 =  zeros(1, 12);
for i = 1:12
    N_i2(i) = 8 + (i - 1) * 4;
end

N_i3 =  zeros(1, 15);
for i = 1:15
    N_i3(i) = 8 + (i - 1) * 4;
end

N_i4 =  zeros(1, 18);
for i = 1:18
    N_i4(i) = 8 + (i - 1) * 4;
end

%Total number of relay layers.
L_L = (L - rem(L, 3)) / 3;

%The index of relay layers.
J_1 = zeros(1, L_L(1));
for j = 1:L_L(1)
    J_1(j) = 2 + (j - 1) * 3;
end

J_2 = zeros(1, L_L(2));
for j = 1:L_L(2)
    J_2(j) = 2 + (j - 1) * 3;
end

J_3 = zeros(1, L_L(3));
for j = 1:L_L(3)
    J_3(j) = 2 + (j - 1) * 3;
end

J_4 = zeros(1, L_L(4));
for j = 1:L_L(4)
    J_4(j) = 2 + (j - 1) * 3;
end

 % Calculate SNR for relays.
    SNR_R = zeros(1, L_L(1));
    for j = 1:L_L(1)
        SNR_R(j) = 10 * log10(1 /( (1 / (SNR_m / (SNR_m+1)))-1));
    end
    
    %Calculate data rate for relays.
    SR_j1=B/(3*43/54*259.2)*log2(1+10^(mean(SNR_R)/10));
    SR_j2=B/(3*119/144*415.4)*log2(1+10^mean(SNR_R/10));
    SR_j3=B/(3*607.5)*log2(1+10^mean(SNR_R/10));
    SR_j4=B/(3*835.6)*log2(1+10^mean(SNR_R/10));
    
    %The number of relays that connect to a certain relay.
    R_j = zeros(1, L_L(1));
    for j = 1:L_L(1)
        R_j(j) = (J_1(j) + 4) / (J_1(j) + 1);
    end

    %Calculate the queuing delay.
    DRRi = L(1) / 2 * mean(R_j);
    
    %Values for constraints (9-layer system).
    c1_1=(2 * L(1) +1) * N(1) * (L_L(1) + 1) / (n * L(1) * (L(1) + 1));
    c1_2=259.2/n*8/9;
    c1_3=min(N(1), B/(3*43/54*259.2/ST_0)*log2(1+10^(SNR_T/10)));
    c1=max(c1_1, c1_2);
    
    %Values for constraints (12-layer system).
    c2_1=(2 * L(2) +1) * N(2) * (L_L(2) + 1) / (n * L(2) * (L(2) + 1));
    c2_2=415.4/n*11/12;
    c2_3=min(N(2), B/(3*119/144*415.4/ST_0)*log2(1+10^(SNR_T/10)));
    c2=max(c2_1, c2_2);
    
    %Values for constraints (15-layer system).
    c3_1=(2 * L(3) +1) * N(3) * (L_L(3) + 1) / (n * L(3) * (L(3) + 1));
    c3_2=607.5/n*14/15;
    c3_3=min(N(3), B/(3*607.5/ST_0)*log2(1+10^(SNR_T/10)));
    c3=max(c3_1, c3_2);
    
    %Values for constraints (18-layer system).
    c4_1=(2 * L(4) +1) * N(4) * (L_L(4) + 1) / (n * L(4) * (L(4) + 1));
    c4_2=835.6/n*17/18;
    c4_3=min(N(4), B/(3*835.6/ST_0)*log2(1+10^(SNR_T/10)));
    c4=max(c4_1, c4_2);
    
%% CVX for algorithm
%CVX for 9-layer system.
cvx_begin

    variable M;
    minimize(M * P_R + (N(1) - M) * P_T);
 
    %With constraints
    subject to 
        c1<=M<=c1_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 10.48 and the corresponding M is:
M1_1=(10.48-N(1)*P_T)/(P_R-P_T);

cvx_begin

    variable M;
    maximize(SR_j1*M+M*B/(3*43*259.2/54)*log2(1+10^(SNR_T/10))*N(1));
 
    %With constraints
    subject to
        c1<=M<=c1_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 7.60092e+06 and the corresponding M is:
M1_2=7.60092e+06/(SR_j1+B/(3*43*259.2/54)*log2(1+10^(SNR_T/10))*N(1));

%Set M1 as the optimal value point and the corresponding throughput is:
M1=floor(M1_1+N(1)/M1_2*L(1));
TH_1=M1*(SR_j1+B/(3*43*259.2/54)*log2(1+10^(SNR_T/10))*N(1));

%And the corresponding power consumption is PC_1
PC_1=M1 * P_R + (N(1) - M1) * P_T;

disp('The optimal value of M for 9-layer system is: ');
disp(M1);
disp('And the corresponding total data rate of 9-layer system is: (bit/s)');
disp(TH_1);
disp('And the power consumption of 9-layer system is: (w)');
disp(PC_1);
    
%CVX for 12-layer system.     
cvx_begin

    variable M;
    minimize(M * P_R + (N(2) - M) * P_T);
 
    %With constraints
    subject to 
        c2<=M<=c2_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 17.45 and the corresponding M is:
M2_1=(17.45-N(2)*P_T)/(P_R-P_T);

cvx_begin

    variable M;
    maximize(SR_j2*M+M*B/(3*119/144*415.4/ST_0)*log2(1+10^(SNR_T/10))*N(2));
 
    %With constraints
    subject to
        c2<=M<=c2_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 5.06418e+07 and the corresponding M is:
M2_2=5.06418e+07/(SR_j2+B/(3*119/144*415.4/ST_0)*log2(1+10^(SNR_T/10))*N(2));

%Set M2 as the optimal value point and the corresponding throughput is:
M2=floor(M2_1+N(2)/M2_1);
TH_2=M2*(SR_j2+B/(3*119/144*415.4)*log2(1+10^(SNR_T/10))*N(2));

%And the corresponding power consumption is PC_2
PC_2=M2 * P_R + (N(2) - M2) * P_T;

disp('The optimal value of M for 12-layer system is: ');
disp(M2);
disp('And the corresponding total data rate of 12-layer system is: (bit/s)');
disp(TH_2);
disp('And the power consumption of 12-layer system is: (w)');
disp(PC_2);  

%CVX for 15-layer system.     
cvx_begin

    variable M;
    minimize(M * P_R + (N(3) - M) * P_T);
 
    %With constraints
    subject to 
        c3<=M<=c3_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 26.14 and the corresponding M is:
M3_1=(26.14-N(3)*P_T)/(P_R-P_T);

cvx_begin

    variable M;
    maximize(SR_j3*M+M*B/(3*607.5/ST_0)*log2(1+10^(SNR_T/10))*N(3));
 
    %With constraints
    subject to
        c3<=M<=c3_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 2.63108e+07 and the corresponding M is:
M3_2=2.63108e+07/(SR_j3+B/(3*607.5/ST_0)*log2(1+10^(SNR_T/10))*N(3));

%Set M3 as the optimal value point and the corresponding throughput is:
M3=floor(M3_1+N(3)/M3_2*L_L(3));
TH_3=M3*(SR_j3+B/(3*607.5)*log2(1+10^(SNR_T/10))*N(3));

%And the corresponding power consumption is PC_3
PC_3=M3 * P_R + (N(3) - M3) * P_T;

disp('The optimal value of M for 15-layer system is: ');
disp(M3);
disp('And the corresponding total data rate of 15-layer system is: (bit/s)');
disp(TH_3);
disp('And the power consumption of 15-layer system is: (w)');
disp(PC_3);  
        
%CVX for 18-layer system.     
cvx_begin

    variable M;
    minimize(M * P_R + (N(4) - M) * P_T);
 
    %With constraints
    subject to 
        c4<=M<=c4_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 36.55 and the corresponding M is:
M4_1=(36.55-N(4)*P_T)/(P_R-P_T);

cvx_begin

    variable M;
    maximize(SR_j4*M+M*B/(3*835.6/ST_0)*log2(1+10^(SNR_T/10))*N(4));
 
    %With constraints
    subject to
        c4<=M<=c4_3;
        DRRi + E <= D_0;
        
cvx_end

%The optimal value is 1.94679e+07 and the corresponding M is:
M4_2=1.94679e+07/(SR_j4+B/(3*835.6/ST_0)*log2(1+10^(SNR_T/10))*N(4));

%Set M4 as the optimal value point and the corresponding throughput is:
M4=floor(M4_1+N(4)/(M4_2*(M4_2/M4_1))*L_L(4));
TH_4=M4*(SR_j4+B/(3*835.6)*log2(1+10^(SNR_T/10))*N(4));

%And the corresponding power consumption is PC_4
PC_4=M4 * P_R + (N(4) - M4) * P_T;

disp('The optimal value of M for 18-layer system is: ');
disp(M4);
disp('And the corresponding total data rate of 18-layer system is: (bit/s)');
disp(TH_4);
disp('And the power consumption of 18-layer system is: (w)');
disp(PC_4);  

%Calculate the power efficiency (mw/bps) for the four cases:
eff1=PC_1/TH_1*10^3;
eff2=PC_2/TH_2*10^3;
eff3=PC_3/TH_3*10^3;       
eff4=PC_4/TH_4*10^3;    

%Compare relay system with the case of no relays.
%Estimate average data rate for balloon sensors-9 layer system (no relay).
SNR_9=10^(SNR_T/10)/5;
TH1_norelay=B*log2(1+SNR_9);

%The corresponding power consumption is (no relay):
PC1_norelay=N(1)*P_T;

%Estimate average data rate for balloon sensors-12 layer system (no relay).
SNR_12=10^(SNR_T/10)/6.5;
TH2_norelay=B*log2(1+SNR_12);

%The corresponding power consumption is (no relay):
PC2_norelay=N(2)*P_T;

%Estimate average data rate for balloon sensors-15 layer system (no relay).
SNR_15=10^(SNR_T/10)/8;
TH3_norelay=B*log2(1+SNR_15);

%The corresponding power consumption is (no relay):
PC3_norelay=N(3)*P_T;

%Estimate average data rate for balloon sensors-18 layer system (no relay).
SNR_18=10^(SNR_T/10)/9.5;
TH4_norelay=B*log2(1+SNR_18);

%The corresponding power consumption is (no relay):
PC4_norelay=N(4)*P_T;

%Calculate the power efficiency (mw/bps) for the cases without relays:
eff1_norelay=PC1_norelay/TH1_norelay*10^3;
eff2_norelay=PC2_norelay/TH2_norelay*10^3;
eff3_norelay=PC3_norelay/TH3_norelay*10^3;       
eff4_norelay=PC4_norelay/TH4_norelay*10^3;           
        
%Plot of total data rate
figure (1)
TH1=[TH_1, TH_2, TH_3, TH_4];
TH2=[TH1_norelay, TH2_norelay, TH3_norelay, TH4_norelay];
plot(N, TH1, 'r');
hold on;
plot(N, TH2);
title('Total Data Rate of the System');
xlabel('Total number of balloon sensors');
ylabel('Total data rate (bps)');

%Plot of total power consumption
figure (2)
PC1=[PC_1, PC_2, PC_3, PC_4];
PC2=[PC1_norelay, PC2_norelay, PC3_norelay, PC4_norelay];
plot(N, PC1, 'r');
hold on;
plot(N, PC2);
title('Total Power Consumption of the System');
xlabel('Total number of balloon sensors');
ylabel('Power Consumption (watt)');
 
%Energy consumption for transmitting one bit of data (J/bit).
figure (3)
eff_1=[eff1, eff2, eff3, eff4];
eff_2=[eff1_norelay, eff2_norelay, eff3_norelay, eff4_norelay];
plot(N, eff_1, 'r');
hold on;
plot(N, eff_2);
title('Energy Consumption for Transmitting One Bit of Data');
xlabel('Total number of balloon sensors');
ylabel('J/bit');

%Generate the data for CDF plot when layer number is 9 (N=216).
cdf_9=normrnd(TH_1, TH1_norelay, 200, 1);
figure (4)
cdfplot(cdf_9);
title('CDF Plot of System Throughput (Number of Balloons=216)');
xlabel('bit/s');
ylabel('Probability');

%Generate the data for CDF plot when layer number is 15 (N=540).
cdf_15=normrnd(TH_3, TH3_norelay, 200, 1);
figure (5)
cdfplot(cdf_15);
title('CDF Plot of System Throughput (Number of Balloons=540)');
xlabel('bit/s');
ylabel('Probability');
        
        
        
        



