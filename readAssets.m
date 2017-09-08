% Generate a manifest from PLY files
clear all; close all; clc;
%load('londress_vox10_plyfiles.mat')
files = dir('c:\Users\Jounsup.Park\Documents\ClientBufferManager\assets');
num_asset = size(files,1)-2;
rt = [3 5 15 30 55;
    3.5 5 8 16 27;
    3.5 6 9 18 30;
    3.5 6 11 20 37.1;
    3.9 6 13 27 42.7]*1e6;
depthList = [9 9 10 10 10];
fps = 30; gofsize = 4;
A = Asset;

for asset = 1:num_asset
    r = rt(asset,:);
    Afiles = dir([files(asset+2).folder '\' files(asset+2).name]);
    num_rep = size(Afiles,1) - 2;
    
    % Asset class
    %A(asset).manifest = manifest;
    A(asset).manifest.startTime = 0;
    A(asset).manifest.endTime = A(asset).manifest.startTime + A(asset).manifest.duration;
    A(asset).manifest.segmentDuration = 1;
    A(asset).manifest.representationCount = num_rep;
    A(asset).manifest.Rep = Representations;
    for rr=1:A(asset).manifest.representationCount
        A(asset).manifest.Rep(rr).Bitrate = r(rr);
    end
    
    for representation = 1:num_rep
        Rfiles = dir([Afiles(representation+2).folder '\' Afiles(representation+2).name]);
        depth = depthList(representation); 
        A(asset).manifest.Rep(representation).LOD = depth;
        num_frame = size(Rfiles,1) - 2;
        A(asset).manifest.duration = num_frame/fps;
        dim = 3; num = 16; num_tile = num^dim;
        tileindex = (1:num_tile)*(2^depth/num)^dim;
        alpha = 1; beta = 1;
        octreeBytes = zeros(1, num_frame);
        for frame = 1:num_frame
            %A(asset).submanifest(frame) = submanifest;
            A(asset).submanifest(frame).gofCount = num_frame/gofsize;
            for gg=1:A(asset).submanifest(frame).gofCount
                A(asset).submanifest(frame).gof(gg) = gof;
                A(asset).submanifest(frame).gof(gg).startTime = 0;
                A(asset).submanifest(frame).gof(gg).Duration = 4/fps;
                A(asset).submanifest(frame).gof(gg).frameCount = 4;
                A(asset).submanifest(frame).gof(gg).tileCount = num_tile;
            end

            A(asset).submanifest(frame).Rep(representation) = subRepresentations;
            for gg=1:A(asset).submanifest(frame).gofCount
                for tt=1:num_tile
                    A(asset).submanifest(frame).Rep(representation).Gof(gg).Tile(tt).mortonCode = tileindex(tt);
                    A(asset).submanifest(frame).Rep(representation).Gof(gg).Tile(tt).byteOffset = 0;
                end
            end
            frame
            tic
            infile = [Rfiles(frame+2).folder '\' Rfiles(frame+2).name];
            % Read file and remove invalid points, if any.
            vpc(frame) = voxelizedPointCloud(infile);
            vpc(frame) = vpc(frame).removeInvalidPoints();

            % Set transform to map tightest bounding cube into [0,cubeWidth]^3.
            cubeWidth = 2^depth-1;
            vpc(frame).Depth = depth;
            vpc(frame) = vpc(frame).setTransform(cubeWidth);

            % And use the transform to map world to frame coordinates.
            %vpc(frame) = vpc(frame).worldToFrame();

            % Create Morton Codes and sort.
            vpc(frame) = vpc(frame).mortonizeAndSort();

            % Voxelize.
            vpc(frame) = vpc(frame).voxelize();

            % Create octree occupancy bytes in breadth first order.
            ot(frame) = octreeClass(vpc(frame).Morton,vpc(frame).Depth);
            %octreeBytes(frame) = ot.occupancyCodes();
            toc
        end

        for frame = 1:num_frame
            % Count occupied tile
            for dd = 1:depth
                tileCount(frame, dd) = length(ot(frame).OccupancyCode{dd});
                C(frame, :, dd) = r/tileCount(frame,dd);
                U(frame, :, dd) = alpha*log(beta*C(frame, :, dd));
            end
            A(asset).submanifest(frame).Rep(representation).C = C(frame,representation,:);
            A(asset).submanifest(frame).Rep(representation).U = U(frame,representation,:);
            A(asset).submanifest(frame).Rep(representation).vpc = vpc(frame);
        end
    end
end

figure(1); bar(tileCount);
figure(2); mesh(tileCount); xlabel('tile'); ylabel('frame'); zlabel('Num of tile');
figure(3); plot(tileCount);