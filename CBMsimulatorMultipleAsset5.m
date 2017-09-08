%% combined simulator - LoD and multiple assets
close all; clear all; clc;
%% Manifest
% Video rates
r = [3 6 10 16 27]*1e6;
% Read frames - returns occupied tiles - submanifest
load('cameraPath6.mat');
load('Assets.mat');
cameraPath = cameraPath6(:,[2 4 3 5 7 6]);
depth = [2 2 2 2 2];
W = 5; bufGof = W*7;
packetSize = 256; % bytes
numberofPacket = 288; % initial number of packet for one segment
segmentDuration = 4/30; % chunk duration (sec)
worldTime = 0; % global time counter
numberofSegment = length(cameraPath); % number of segments for simulation
pb = Playback;
playOut = 0; % timing for play out

%% Assets
orgin = [0 0 -2000; 0 2000 0; 0 0 0; 0 -2000 0; 2000 0 0];
LoD = [1 1 1 1 1]; %[contant/dist*min{dist,displaypixels}]^2
num_assets = size(orgin,1);

% Parameters for assets
for aa=1:num_assets
    tileCount(aa) = (2^depth(aa))^3; % number of occupied tiles
    Umatrix{aa} = zeros(tileCount(aa), length(r));
    Cmatrix{aa} = zeros(tileCount(aa), length(r));
    Pmatrix{aa} = zeros(tileCount(aa), bufGof);
    stateMatrix{aa} = zeros(tileCount(aa), bufGof);
    cb(aa) = clientBufferState(0,0,segmentDuration,[],worldTime);
    bufState{aa} = zeros(sum(tileCount(aa)),60);
    displayTile{aa} = zeros(1,tileCount(aa));
end
% Buffer class
%cb = clientBufferCT(0,0,segmentDuration,[],worldTime);
%% Network model
% Model based network throughput generation
networkModel = [20 15 10 5 3 1 3 5 10 15 20 ...
    20 15 10 5 3 1 3 5 10 15 20 ...
    1 3 5 10 15 20 1 3 5 10 15 20 ...
    5 15 5 15 5 15 5]*3e6;
%networkModel = 60e6;
mu = 8*packetSize./networkModel;

%% Buffer Management algorithm
% state matrix
state = zeros(sum(tileCount)*bufGof, 1);

% Read first segment
R(1) = packetSize*numberofPacket*8/segmentDuration; % initial video rate
ff=1;
% Loop until done
SmoothedChat(1) = 0; f = 1;
playbackCounter = 1;
for ss=1:numberofSegment
    frame = mod(ss,4)+1;
    % user behaviour
    for aa=1:num_assets
        [visibleTile{aa} occupiedTile{aa} tileindex{aa} oxyz{aa} vectorR{aa}]...
            = visibleTiles(A(aa).submanifest(frame).Rep(5).vpc, orgin(aa,:), cameraPath(ss,:), 2^depth(aa));
        p{aa} = visibleTile{aa};
        dist(aa) = sum((cameraPath(ss,1:3)+orgin(aa,:)).^2);
        LoD(aa) = 1e6/dist(aa);
        for tt=1:tileCount(aa)
            for rr=1:length(r)
                Umatrix{aa}(tt,rr) = A(aa).submanifest(frame).Rep(rr).U(depth(aa));
                Cmatrix{aa}(tt,rr) = A(aa).submanifest(frame).Rep(rr).C(depth(aa));
            end
        end
        for bb=1:bufGof
            Perr = 0.1+0.3*bb/bufGof;
            Pmatrix{aa}(:,bb) = (p{aa}*(1-Perr)+(1-p{aa})*Perr).*occupiedTile{aa};
            for rr=1:length(r)
                tempU((bb-1)*tileCount(aa)+1:bb*tileCount(aa),rr) = Umatrix{aa}(:,rr).*Pmatrix{aa}(:,bb)*LoD(aa);
            end
            tempC((bb-1)*tileCount(aa)+1:bb*tileCount(aa),:) = Cmatrix{aa};
        end
        if aa==1, U = tempU; C = tempC;
        else U = [U; tempU]; C = [C; tempC];
        end
    end
    %subplot(8, num_assets, [1 5]);
    fig1 = figure(1); set(fig1, 'Position', [-1000 0 1000 800]);
    scatter3(cameraPath(ss,1),cameraPath(ss,2),cameraPath(ss,3),'mX'); hold on;
    scatter3(cameraPath(ss,4),cameraPath(ss,5),cameraPath(ss,6),'k^');
    for aa=1:num_assets
        if aa==1
            s = 10*ones(length(oxyz{aa}),1);
            c = A(aa).submanifest(frame).Rep(5).vpc.Color/256;
            scatter3(oxyz{aa}(:,3),oxyz{aa}(:,2),oxyz{aa}(:,1),s,c);
            scatter3(oxyz{aa}(vectorR{aa},3), oxyz{aa}(vectorR{aa},2), oxyz{aa}(vectorR{aa},1),'r')
        else
            s = 10*ones(length(oxyz{aa}),1);
            c = A(aa).submanifest(frame).Rep(5).vpc.Color/256;
            scatter3(oxyz{aa}(:,1),oxyz{aa}(:,2),oxyz{aa}(:,3),s,c);
            scatter3(oxyz{aa}(vectorR{aa},1), oxyz{aa}(vectorR{aa},2), oxyz{aa}(vectorR{aa},3),'r')
        end
    end
    xlabel('x'); ylabel('y'); zlabel('z'); axis([-3000 3000 -3000 3000 0 1000]); view(120+37.5,60);
    drawnow; F1(ss) = getframe(fig1); 
    hold;

    % Buffer management
    bufferTargetDuration = min([W ceil(pb.currentTime+1)]);
    downloadTime = 0;
    for pp=1:numberofPacket
        ar = -mu(floor(length(mu)*ss/(numberofSegment+1))+1)*log(rand);
        %ar = -mu*log(rand);
        worldTime(ss) = worldTime(ss) + ar;
        pb.currentTime = worldTime(ss);
        downloadTime = downloadTime + ar; % segment time counter

        % Packet departures - every chunk period
        if floor(worldTime(ss)/segmentDuration) > playOut
            tempPlay = floor(worldTime(ss)/segmentDuration) - playOut;
            for tt=1:tempPlay
                playbackCounter = playbackCounter + 1;
                for aa=1:num_assets
                    if cb(aa).o>0, cb(aa) = cb(aa).segmentout(worldTime(ss)); end
                    tempState{aa} = zeros(1,tileCount(aa)*bufGof);
                    stateIndex = sum(tileCount(1:aa-1))*bufGof+(tileCount(aa)+1:tileCount(aa)*bufGof);
                    tempState{aa}(1:end-tileCount(aa)) = state(stateIndex);
                    tempIndex = mod(playbackCounter-2+bufGof,60)+1;
                    tempState{aa}(end-tileCount(aa)+1:end) = bufState{aa}(:,tempIndex);
                    state(sum(tileCount(1:aa-1))*bufGof+(1:tileCount(aa)*bufGof)) = tempState{aa};
        %             tempState2 = zeros(1,tileCount*bufGof);
        %             tempState2(1:end-tileCount2) = state(tileCount2+tileCount*bufGof+1:end);
        %             state(tileCount*bufGof+1:end) = tempState2;
                    playOut = floor(worldTime(ss)/segmentDuration);
                    %stateMatrix{aa} = reshape(state(sum(tileCount(1:aa-1))*bufGof+(1:tileCount(aa)*bufGof)), [tileCount(aa) bufGof]);
                    %tempIndex2 = mod((ss:ss+bufGof-1)-1,60)+1;
                    %bufState{aa}(:,tempIndex2) = stateMatrix{aa};
                end
            end
        end
    end
%     for aa=1:num_assets
%         cb(aa) = cb(aa).segmentin(worldTime(ss),stateMatrix{aa});
%     end
    
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
            for cc=1:length(cb), mcbo(cc)=cb(cc).o; end
            window = max([floor(bufferTargetDuration)-max(mcbo) 1]);
            BW = SmoothedChat(ss+1)/window;
    end
    %% Utility Maximization and Make a request
    oristate = state;
    state = TBRS_ARv2(BW, sum(tileCount)*bufGof, length(r), U, C, state);
    for aa=1:num_assets
        displayTile{aa} = occupiedTile{aa} | displayTile{aa};
        stateMatrix{aa} = reshape(state(sum(tileCount(1:aa-1))*bufGof+(1:tileCount(aa)*bufGof)), [tileCount(aa) bufGof]);
        cb(aa) = cb(aa).segmentin(worldTime(ss),stateMatrix{aa});
        tempIndex = mod((playbackCounter:playbackCounter+bufGof-1)-1,60)+1;
        bufState{aa}(:,tempIndex) = stateMatrix{aa};
        fig2 = figure(2); set(fig2, 'Position', [0 0 1920 1000]);
        subplot(7,num_assets,4*num_assets+aa);  
        plot(worldTime,Chat,'r'); ylabel('bps'); title('Estimated Throughput');
        subplot(7,num_assets,[0*num_assets+aa 1*num_assets+aa]); 
        bar3(stateMatrix{aa}(find(displayTile{aa}),:)); axis([0 bufGof 0 sum(displayTile{aa}) 0 length(r)]); view(30,37.5); drawnow;
        xlabel('<-- Playback'); ylabel('Tile index'); zlabel('Representation'); title('Window');
        %F2(ss) = getframe;
        %figure(3); 
        subplot(7,num_assets,[2*num_assets+aa 3*num_assets+aa]); 
        bar3(bufState{aa}(find(displayTile{aa}),:)); axis([0 60 0 sum(displayTile{aa}) 0 length(r)]); view(30,37.5); drawnow;
        xlabel('Frame'); ylabel('Tile index'); zlabel('Representation'); title('Buffer Status');
        %F3(ss) = getframe;
        for tt=1:tileCount(aa)
            if stateMatrix{aa}(tt,1)>0
                utility{aa}(ss,tt)=p{aa}(tt).*Umatrix{aa}(tt,stateMatrix{aa}(tt,1));
            else
                utility{aa}(ss,tt)=0;
            end
        end
        subplot(7,num_assets,5*num_assets+aa);  
        plot(worldTime,sum(utility{aa},2),'g'); 
        xlabel('sec'); ylabel('Utility'); %axis([0 ss 0 max(sum(utility{aa},2))]);
        subplot(7,num_assets,6*num_assets+aa);  
        cbo(aa,ss)=cb(aa).o;
        plot(worldTime,cbo(aa,:),'b'); 
        xlabel('sec'); ylabel('Buffer Occupancy'); %axis([0 ss 0 max(cbo(aa,:))]);
    end
    F2(ss) = getframe(fig2);
    R(ss+1) = max([1 BW]);
    numberofPacket = floor(R(ss+1)*segmentDuration/(packetSize*8));
    worldTime(ss+1) = worldTime(ss);
    
 
    %% Output file generation
    for aa=1:num_assets
        visibleVpc = voxelizedPointCloud;
        visibleVpc.Depth = depth(aa);
        count = 0; scount = 1;
        for vv=1:tileCount(aa)
            clear fi;
            rep = stateMatrix{aa}(vv,1);
            if rep>0
                if (vv)==1
                    fi = find(tileindex{aa}((vv))>=A(aa).submanifest(frame).Rep(rep).vpc.Morton);
                else
                    fi = find(tileindex{aa}((vv))>=A(aa).submanifest(frame).Rep(rep).vpc.Morton & tileindex{aa}((vv)-1)<=A(aa).submanifest(frame).Rep(rep).vpc.Morton);
                end
                visibleVpc.XLimits = A(aa).submanifest(frame).Rep(rep).vpc.XLimits;
                visibleVpc.YLimits = A(aa).submanifest(frame).Rep(rep).vpc.YLimits;
                visibleVpc.ZLimits = A(aa).submanifest(frame).Rep(rep).vpc.ZLimits;
                visibleVpc.ColorSpace = 'RGB';
                visibleVpc.FrameToWorldScale = A(aa).submanifest(frame).Rep(rep).vpc.FrameToWorldScale;
                visibleVpc.FrameToWorldTranslation = A(aa).submanifest(frame).Rep(rep).vpc.FrameToWorldTranslation;
                visibleVpc.CubeWidth = 1023;
                count = count + length(fi);
                visibleVpc.Location(scount:count,:) = A(aa).submanifest(frame).Rep(rep).vpc.Location(fi,:);
                visibleVpc.Color(scount:count,:) = A(aa).submanifest(frame).Rep(rep).vpc.Color(fi,:);
                visibleVpc.Morton(scount:count) = A(aa).submanifest(frame).Rep(rep).vpc.Morton(fi);
                %visibleVpc.Asave(scount:count,scount:count) = diag(length(fi));
                visibleVpc.Isave(scount:count) = 1;

                scount = count+1;
            end
        end
        visibleVpc.Count = count;
        if count>0, visibleVpc.write(['Asset' num2str(aa) 'frame' num2str(ss) '.ply'],'ascii'); end
    end
end
%% Performance evaluation
for x=1:10*ss, 
    %fig3=figure(3); set(fig3,'Position', [-1000 0 1000 800]); imshow(F1(mod(x-1,ss)+1).cdata); drawnow;
    fig4=figure(4); drawnow; set(fig4,'Position', [0 0 1920 1000]); imshow(F2(mod(x-1,ss)+1).cdata);  
end