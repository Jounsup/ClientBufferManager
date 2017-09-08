% Client Buffer Manager simulation - Sever, Network and Client buffer
clear all; close all; clc;

worldTime = 0; % global time counter
playOut = 0; % timing for play out
prefetch = 1; % number of segments to prefetch

%% Server - video chunks manifest
r = [3 6 10 16 27]*1e6;
load('CUwDepth.mat');
dd = 4;
bufGof = 5*7;
tileCount = tileCountWdepth(1,dd);
Umatrix = zeros(tileCount, length(r));
Cmatrix = zeros(tileCount, length(r));
Pmatrix = zeros(tileCount, length(r));
for tt=1:tileCount
    Umatrix(tt,:) = UwDepth(1,:,dd);
    Cmatrix(tt,:) = CwDepth(1,:,dd);
end
p = [zeros(1,floor(tileCount/4)) ones(1,floor(tileCount/2)) zeros(1,floor(tileCount/4))];
grd = 1;
for bb=1:bufGof
    if mod(bb,5)==0, grd = grd+1; end
    Pmatrix(:,bb) = p/sum(p)/grd;
    if mod(bb,2)==0,    p = conv(p,[0 p],'same'); p = p/sum(p);
    else                p = conv(p,[p 0],'same'); p = p/sum(p);
    end
end
figure(3); bar3(Pmatrix);
for bb=1:bufGof
    for rr=1:length(r)
        U((bb-1)*tileCount+1:bb*tileCount,rr) = Umatrix(:,rr).*Pmatrix(:,bb);
    end
    C((bb-1)*tileCount+1:bb*tileCount,:) = Cmatrix;
end
FrameToWorldScale = 0;
FrameToWorldRotation = 0;
FrameToWorldTranslation = 0;
numberofSegment = 1000; % number of segments for simulation
segmentDuration = 4/30; % chunk duration (sec)
startTime = 0;
duration = numberofSegment * segmentDuration;
endTime = startTime + duration;
startSegment = 0;
degreesPerTileAt1m = 0;
representationCount = length(r);
URLbase = 0;
Rep = r;
packetSize = 256; % bytes
numberofPacket = 488; % initial number of packet for one segment

%% Network model
% Model based network throughput generation
% networkModel = [20 15 10 5 3 1 3 5 10 15 20 ...
%     20 15 10 5 3 1 3 5 10 15 20 ...
%     1 3 5 10 15 20 1 3 5 10 15 20 ...
%     5 15 5 15 5 15 5]*1e6;
networkModel = 30e6;
mu = 8*packetSize./networkModel;

%% Client buffer model
cb(1) = clientBufferCT(0,0,segmentDuration,[],worldTime);

% initialize
downloadBytes = 0;
downloadTime = 0;

% Get Asset
m1 = manifest;
m2 = submanifest;
A = Asset;
A.manifest = m1;
A.submanifest = m2;

% Select clip to play and speed
AssetClipStartTime = A.manifest.startTime;
AssetClipEndTime = A.manifest.endTime;
AssetClipSpeed = 1;

% state matrix
state = zeros(tileCount*bufGof, 1);

% Read first segment
R(1) = packetSize*numberofPacket*8/segmentDuration; % initial video rate
StartSegment = floor((AssetClipStartTime-A.manifest.startTime)/A.manifest.duration);
%downloadBytes = downloadBytes + A.submanifest.Rep;

% Prefetch
for ff=1:prefetch
    for pf = 1:numberofPacket
        ar = -mu(1)*log(rand); % exprnd(mu(1));
        worldTime = worldTime + ar;
    end
    cb(ff+1) = cb(ff).segmentin(worldTime,R(1));
end

% Loop until done
pb = Playback;
SmoothedChat(1) = 0; f = 0.5;

for ss=1:numberofSegment-1
    ss
    bufferTargetDuration = min([5 ceil(pb.currentTime+1)]);
    downloadTime = 0;
    %if mod(ss,100) == 0, mu = 0.1*rand(); end
    for pp=1:numberofPacket
        % Packet arrivals - every segment 
        %wB = 0.3;
        %wA = mu(floor(length(mu)*ss/numberofSegment)+1)/gamma(1+1/wB);
        ar = -mu(floor(length(mu)*ss/numberofSegment)+1)*log(rand);
        %ar = wblrnd(wA,wB); % Weibull
        %ar = exprnd(mu(floor(4*ss/num_seg)+1)); % Exponential
        %ar = Ti(pp+1)-Ti(pp); % Time-varying Poisson process
        worldTime = worldTime + ar;
        pb.currentTime = worldTime;
        downloadTime = downloadTime + ar; % segment time counter

        % Packet departures - every chunk period
        if floor(worldTime/segmentDuration) > playOut
            cb(ff+1) = cb(ff).segmentout(worldTime);
            tempState = zeros(1,tileCount*bufGof);
            tempState(1:end-tileCount) = state(tileCount+1:end);
            state = tempState;
            playOut = floor(worldTime/segmentDuration);
            ff = ff + 1;
        end
        % Buffer occupancy update
    end
    cb(ff+1) = cb(ff).segmentin(worldTime,R(ss));
    ff = ff + 1;
    %% Rate adaptation algorithm
    Chat(ss) = R(ss)*segmentDuration/downloadTime;
    SmoothedChat(ss+1) = f*Chat(ss) + (1-f)*SmoothedChat(ss);
    
    switch(4)
        case 1
            % Throughput based algorithm
            %ri = sum(mean(Chat(max([1 (ss-5)]):ss))>r); 
            BW = sum(SmoothedChat(ss+1)>r); 
        case 2
            % Buffer based algorithm
            BW = sum(400000*cb(ff).o>r); 
        case 3
            % Our algorithm with buffer overflow protection
            if cb(ff).o>floor(bufferTargetDuration/segmentDuration)
                BW = sum((Chat(ss)*(cb(ff).o-floor(bufferTargetDuration/segmentDuration)+1))>r*segmentDuration)
            else BW = sum((Chat(ss)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
            end
        otherwise 
            % Our algorithm
            %ri = sum((Chat(ss)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
            %BW = (SmoothedChat(ss+1)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))/segmentDuration;
            %BW = (SmoothedChat(ss+1)/(floor(bufferTargetDuration)-cb(ff).o))/segmentDuration;
            BW = SmoothedChat(ss+1)/(floor(bufferTargetDuration)-(cb(ff).o));
    end
    %% Utility Maximization and Make a request
    %state = reshape(stateMatrix(:,1:bufferTargetDuration), [1 tileCount*bufferTargetDuration]);
    %U = reshape(Umatrix, [tileCount*bufferTargetDuration length(r)]);
    %C = reshape(Cmatrix, [tileCount*bufferTargetDuration length(r)]);
    %P = reshape(Pmatrix(:,1:bufferTargetDuration), [1 tileCount*bufferTargetDuration]);
    state = TBRS_ARv2(BW, tileCount*bufGof, 5, U, C, state);
    stateMatrix = reshape(state(1:tileCount*bufGof), [tileCount bufGof]);
    figure(1); bar3(stateMatrix'); axis([0 tileCount 0 bufGof 0 length(r)]); drawnow;
    F(ss) = getframe;
    R(ss+1) = max([1 BW]);
    %R(ss+1) = (SmoothedChat(ss+1)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))/segmentDuration;
    numberofPacket = floor(R(ss+1)*segmentDuration/(packetSize*8));
    for tt=1:tileCount
        if stateMatrix(tt,1)>0
            utility(ss,tt)=(Pmatrix(tt,1)>0).*Umatrix(tt,stateMatrix(tt,1));
        else
            utility(ss,tt)=0;
        end
    end
end

for ll=1:size(cb,2)-1, plot_t(ll) = cb(ll).tt; plot_o(ll) = cb(ll).o; end
figure(2); 
subplot(4,1,1); 
plot(plot_t, plot_o); ylabel('sec'); title('Buffer Occupancy'); 
text(1,1,['Stall' num2str(cb(end).st)]);
subplot(4,1,2); 
plot(segmentDuration*(0:length(Chat)-1),Chat,'r'); ylabel('bps'); title('Estimated Throughput'); 
text(5,1e7,['Average' num2str(mean(Chat),'%1.2e')]);
subplot(4,1,3); 
plot(segmentDuration*(0:length(R)-1),R,'k'); ylabel('bps'); title('Bitrate'); 
text(5,1e7,['Average' num2str(mean(R),'%1.2e')]);
subplot(4,1,4);
plot(segmentDuration*(0:length(utility(:,1))-1),sum(utility,2),'g'); xlabel('sec'); ylabel('utility'); title('Utility');
text(5,250,['Average' num2str(mean(sum(utility,2)),'%3.0d')]);