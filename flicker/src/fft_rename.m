Fs = 240; %Sampling Frequency
T = 1/Fs;
L = 4868; %Signal length
X = table2array(x)
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1)