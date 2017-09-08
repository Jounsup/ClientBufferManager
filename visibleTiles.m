% % 7/28/17 discussion
% % 1. combining simulators
% % 2. looping feature (5sec window, larger buffer, updating existing data)
% % 3. multiple sources
% % 4. make a list of simulations (plots, comparisons)
% 
% clear all; close all; clc;
% 
% %load('londress_vox10_plyfiles.mat');
% load('londress1_vox10_plyfiles.mat');
function [visibleTile occupiedTile tileindex oxyz vectorR] = visibleTiles(vpc, orgin, cameraPath, dd)
    xyz = mortonToXyz(vpc.Morton,10);
    oxyz(:,1) = int16(xyz(:,1)) - orgin(1);
    oxyz(:,2) = int16(xyz(:,3)) - orgin(2);
    oxyz(:,3) = int16(xyz(:,2)) - orgin(3);
    %load('cameraPath4.mat');
    %cameraPath = cameraPath4(:,[2 4 3]);
    delta = 0.1; stepTh = 0.0036; stepPhi = 0.0097;

    cameraPosition = cameraPath(1:3);
    cameraDirection = cameraPath(4:6);
    [cdTh cdPhi cdR] = cart2sph(cameraDirection(1)-cameraPosition(1),cameraDirection(2)-cameraPosition(2),cameraDirection(3)-cameraPosition(3));
    angleWidth = 1/3;
    angleHeight = 1/4;

    nxyz(:,1) = oxyz(:,1) - cameraPosition(1);
    nxyz(:,2) = oxyz(:,2) - cameraPosition(2);
    nxyz(:,3) = oxyz(:,3) - cameraPosition(3);

    sXyz = single(nxyz);
    [th phi r] = cart2sph(sXyz(:,1),sXyz(:,2),sXyz(:,3));

    q = 100;
    %stepTh = delta*(max(th)-min(th))/q+(1-delta)*stepTh
    %stepPhi = delta*(max(phi)-min(phi))/q+(1-delta)*stepPhi
    q1 = floor(1/stepTh);
    q2 = floor(1/stepPhi);
    qth = floor(th*q1)/q1;
    qphi = floor(phi*q2)/q2;
    lengthQth = int16((max(qth)-min(qth))*q1+1);
    lengthQphi = int16((max(qphi)-min(qphi))*q2+1);
    lengthQth = ceil(angleWidth/stepTh)+1;
    lengthQphi = ceil(angleHeight/stepPhi)+1;
    minR = 1e6*ones(lengthQth, lengthQphi);
    storeR = zeros(lengthQth, lengthQphi);
    indiR = zeros(lengthQth, lengthQphi);

    [sortR indexR] = sort(r,'descend');

    cc = 0;
    while sum(sum(indiR))<lengthQth*lengthQphi & cc<length(r)
        cc = cc + 1;
        ll = indexR(cc);
        if qth(ll)<cdTh+angleWidth/2 & qth(ll)>cdTh-angleWidth/2 & qphi(ll)<cdPhi+angleHeight/2 & qphi(ll)>cdPhi-angleHeight/2
            xa = int16((qth(ll)-cdTh+angleWidth/2)*q1+1);
            ya = int16((qphi(ll)-cdPhi+angleHeight/2)*q2+1);
            if minR(xa,ya)>r(ll)
                minR(xa,ya) = r(ll);
                storeR(xa,ya) = ll;
                indiR(xa,ya) = 1;
            end
        end
    end

    tempR = reshape(storeR, [1 lengthQth*lengthQphi]);
    [tempValue tempIndex] = sort(tempR,'descend');
    count = sum(tempR>0);
    vectorR = tempValue(1:count);

%     figure; 
% %     scatter3(xyz(:,3),xyz(:,1),xyz(:,2),'.'); hold on;
% %     scatter3(xyz(vectorR,3), xyz(vectorR,1), xyz(vectorR,2), 'r')
%     scatter3(oxyz(:,1),oxyz(:,2),oxyz(:,3),'.'); hold on;
%     scatter3(oxyz(vectorR,1), oxyz(vectorR,2), oxyz(vectorR,3), 'r')
%     scatter3(cameraPosition(2),cameraPosition(1),cameraPosition(3),'mX')
%     scatter3(cameraDirection(1),cameraDirection(2),cameraDirection(3),'k^')
%     xlabel('x'); ylabel('y'); zlabel('z'); axis([-3000 3000 -3000 3000 0 1000]);
%     F = getframe; drawnow;
%     hold;
%     figure(4); drawnow;
%     scatter3(qth, qphi,r,'.'); hold on; 
%     scatter3(qth(vectorR), qphi(vectorR), r(vectorR), 'r')
%     xlabel('\theta'); ylabel('\phi'); zlabel('r');
%     hold;

    % extract the visible tiles from visible voxels
    depth = 10;
    dim = 3; num = dd; num_tile = num^dim;
    tileindex = (1:num_tile)*(2^depth/num)^dim;

    mCode = xyzToMorton(xyz(vectorR,:),depth);

    temp_count = 0; temp_count2 = 0;
    visibleTile = zeros(1, num_tile);
    occupiedTile = zeros(1, num_tile);
    for ii=1:num_tile
        num_voxel(ii) = sum(tileindex(ii)>=mCode) - temp_count;
        occ_voxel(ii) = sum(tileindex(ii)>=vpc.Morton) - temp_count2;
        temp_count = sum(num_voxel);
        temp_count2 = sum(occ_voxel);
        if num_voxel(ii) > 0, visibleTile(ii) = 1; end
        if occ_voxel(ii) > 0, occupiedTile(ii) = 1; end
    end

%     visibleVpc = voxelizedPointCloud;
%     visibleVpc.Depth = depth;
%     visibleVpc.XLimits = vpc.XLimits;
%     visibleVpc.YLimits = vpc.YLimits;
%     visibleVpc.ZLimits = vpc.ZLimits;
%     visibleVpc.ColorSpace = 'RGB';
%     visibleVpc.FrameToWorldScale = 0.9804;
%     visibleVpc.FrameToWorldTranslation = vpc(1).FrameToWorldTranslation;
%     visibleVpc.CubeWidth = 1023;
%     count = 0; scount = 1;
%     for vv=1:num_tile
%         clear fi;
%         if visibleTile(vv)==1
%             if vv==1,fi = find(tileindex(vv)>=vpc.Morton);
%             else fi = find(tileindex(vv)>=vpc.Morton & tileindex(vv-1)<=vpc.Morton);
%             end
%             count = count + length(fi);
%             visibleVpc.Location(scount:count,:) = vpc.Location(fi,:);
%             visibleVpc.Color(scount:count,:) = vpc.Color(fi,:);
%             visibleVpc.Morton(scount:count) = vpc.Morton(fi);
%             %visibleVpc.Asave(scount:count,scount:count) = diag(length(fi));
%             visibleVpc.Isave(scount:count) = 1;
% 
%             scount = count+1;
%         end
%     end
%     visibleVpc.Count = count;
%     visibleVpc.write(['newLongdress' num2str(pp) '.ply'],'ascii')

end