%% Client Buffer Manager simulation - Sever, Network and Client buffer
clear all; close all; clc;

global_t = 0; % global time counter
play_t = 0; % timing for play out

%% Server - video chunks
num_seg = 3000; % number of segments for simulation
preT = 10; % number of segments to prefetch
pac_size = 256; % byte
num_pac = 488; % initial number of packet for one segment
T = 4/30; % chunk duration (sec)
R(1) = pac_size*num_pac*8/T; % initial video rate

%% Network model
% Model based network throughput generation
%mu = 0.1*[0.001 0.0006 0.0007 0.0008 0.0009 0.001 0.002 0.003 0.004 0.005 0.006 0.0006 0.0007 0.001 0.001 0.001 0.0006 0.001 0.0007 0.0008 0.0009 0.004 0.002 0.003 0.0006 0.0007 0.003 0.001];
%r = [3e5 7e5 1.5e6 2.5e6 3.5e6];
r = [3 6 10 16 27]*1e6;
%r = [3.9 6 13 27 42.7]*1e6;
networkModel = [20 15 10 5 3 1 3 5 10 15 20 ...
    20 15 10 5 3 1 3 5 10 15 20 ...
    1 3 5 10 15 20 1 3 5 10 15 20 ...
    5 15 5 15 5 15 5]*1e6;
% networkModel = 3.5e6;
mu = 8*pac_size./networkModel;

%% Client buffer model
cb(1) = clientBufferCT(0,0,T,[],global_t);

targetB = 5;

% Prefetch
for ff=1:preT
    for pf = 1:num_pac
        %ar = exprnd(mu(1));
        ar = -mu(1)*log(rand);
        global_t = global_t + ar;
    end
    cb(ff+1) = cb(ff).segmentin(global_t,R(1));
end

% Time-varying Poisson Process
rate = max([zeros(1,1000000);conv(randn(1,1000000),ones(1,100000),'same')]);
%[Ti ti]=PechePourPoisson(rate,0.0001);

% After play
for ss=1:num_seg-1,
    seg_t = 0;
    %if mod(ss,100) == 0, mu = 0.1*rand(); end
    for pp=1:num_pac,
        % Packet arrivals - every segment 
        wB = 0.3;
        wA = mu(floor(length(mu)*ss/num_seg)+1)/gamma(1+1/wB);
        %ar = wblrnd(wA,wB); % Weibull
        %ar = -mu(1)*log(rand);
        ar = -mu(floor(length(mu)*ss/num_seg)+1)*log(rand);
        %ar = exprnd(mu(floor(4*ss/num_seg)+1)); % Exponential
        %ar = Ti(pp+1)-Ti(pp); % Time-varying Poisson process
        global_t = global_t + ar;
        seg_t = seg_t + ar; % segment time counter

        % Packet departures - every second
        if floor(global_t/T) > play_t, 
            cb(ff+1) = cb(ff).segmentout(global_t);
            play_t = floor(global_t/T);
            ff = ff + 1;
        end
        % Buffer occupancy update
    end
    cb(ff+1) = cb(ff).segmentin(global_t,R(ss));
    ff = ff + 1;
    %% Rate adaptation algorithm
    est_th(ss) = R(ss)*T/seg_t;
    switch (2)
        case 1
            % Throughput based algorithm
            %R(ss+1) = mean(est_th(ss:max([1 (ss-5)])));
            R(ss+1) = 0.5*mean(est_th(max([1 (ss-5)]):ss));
            ri = sum(R(ss+1)>r); 
        case 2
            % Buffer based algorithm
            R(ss+1) = 45000*cb(ff).o;
            ri = sum(R(ss+1)>r*T); 
        case 3
            % Our algorithm (constant 5sec buffer occupancy)
            %if cb(ff).o>5, ri = sum((mean(est_th(max([1 (ss-5)]):ss))*(cb(ff).o-3))>r);
            %if cb(ff).o>4, ri = sum((est_th(ss)*(cb(ff).o-4))>r);, 
            %if cb(ff).o>floor(targetB/T), ri = sum((est_th(ss)*(cb(ff).o-floor(targetB/T)+1))>r*T)
            %else ri = sum((est_th(ss)/(floor(targetB/T)-cb(ff).o))>r*T);
            %end
            %ri = sum((est_th(ss)/(5-cb(ff).o))>r);
            ri = sum((est_th(ss)/(floor(targetB/T)-cb(ff).o))>r*T);
        otherwise 
            % Our algorithm
            %ri = sum((Chat(ss)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
            ri = sum((SmoothedChat(ss+1)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
    end
    R(ss+1) = r(max([1 ri]));
    num_pac = floor(R(ss+1)*T/(pac_size*8));
end

for ll=1:size(cb,2)-1, plot_t(ll) = cb(ll).tt; plot_o(ll) = cb(ll).o; end
figure(1); 
subplot(3,1,1); plot(plot_t, plot_o*T); ylabel('Num of GoFs'); title('Buffer Occupancy'); text(5,max(plot_o*T),['Stall' num2str(cb(end).st)]);
subplot(3,1,2); plot(T*(1:ss),est_th,'r'); ylabel('bps'); title('Estimated Throughput'); text(5,max(est_th),['Average' num2str(mean(est_th),'%1.2e')]);
subplot(3,1,3); plot(T*(1:ss),R(1:end-1),'k'); xlabel('sec'); ylabel('bps'); title('Bitrate'); text(5,mean(R),['Average' num2str(mean(R),'%1.2e')]);