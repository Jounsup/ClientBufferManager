%% Test mortonToOctree and octreeToMorton.
clear all
close all
clc
% load('londress_vox10_plyfiles.mat')
files = dir('C:\Users\Jounsup.Park\Documents\ClientBufferManager\longdress');
depth = 10;
for rep = 1:5
    rep
    tic
    infile = [files(rep+2).folder '\' files(rep+2).name];
    % Read file and remove invalid points, if any.
    vpc(rep) = voxelizedPointCloud(infile);
    vpc(rep) = vpc(rep).removeInvalidPoints();

    % Set transform to map tightest bounding cube into [0,cubeWidth]^3.
    cubeWidth = 2^depth-1;
    vpc(rep).Depth = depth;
    vpc(rep) = vpc(rep).setTransform(cubeWidth);

    % And use the transform to map world to frame coordinates.
    %vpc(rep) = vpc(rep).worldToFrame();

    % Create Morton Codes and sort.
    vpc(rep) = vpc(rep).mortonizeAndSort();

    % Voxelize.
    vpc(rep) = vpc(rep).voxelize();
    toc
end