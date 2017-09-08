clear all; close all; %clc;
%% Tile-based rate selection algorithm (Bell lab)
% parameters
I = 64; % number of tiles
Q = 5; % number of representations
x = zeros(I,Q); % selection parameter
U = zeros(I,Q); % marginal utility
P = ones(I,Q); % probability of view
pro = max([0.2*ones(I,1) rand(I,1)]')'; 
for ii=1:I, P(ii,:) = pro(ii)*P(ii,:); end
C = zeros(I,Q); % marginal cost
BW = 1e6; % bandwidth

% Videos
Rv = 1e6*rand(I,Q);
nv = length(Rv); [Vn Nv] = size(Rv);
tempA = max(Rv')'; A = 1e4; 
alpha1 = 1./log(tempA/A); beta1 = tempA/A;
for uu=1:length(alpha1), 
    oriu(uu,:) = alpha1(uu).*log(beta1(uu).*Rv(uu,:)./tempA(uu)); 
end
for ll=1:Nv, 
    if ll==1, U(:,ll) = oriu(:,ll); C(:,ll) = Rv(:,ll);
    else U(:,ll) = oriu(:,ll)-oriu(:,ll-1); C(:,ll) = Rv(:,ll) - Rv(:,ll-1); end 
end % marginal utility

% Intitial value
T = zeros(1,I);
current_BW = 0; accum_U = 0;
uocr = U.*P./C; % Utility over cost ratio
uocr_vector = reshape(uocr,[1 I*Q]);
Rv_vector = reshape(Rv,[1 I*Q]);
U_vector = reshape(U,[1 I*Q]);
C_vector = reshape(C,[1 I*Q]);
P_vector = reshape(P,[1 I*Q]);
x_vector = reshape(x,[1 I*Q]);
[uocr_sort uocr_index] = sort(uocr_vector,'descend');

% Base group Iteration
for ii=1:length(uocr_index),
    if current_BW(ii) <= BW, 
        current_BW(ii+1) = current_BW(ii) + C_vector(uocr_index(ii));
        accum_U(ii+1) = accum_U(ii) + U_vector(uocr_index(ii))*P_vector(uocr_index(ii));
        x_vector(uocr_index(ii)) = 1;
        basex = reshape(x_vector, [I Q]);
    else break; end
end
baseU = sum(sum(basex.*U,2).*pro)
figure(1); grid on; xlabel('Available Rate'); ylabel('Utility'); hold on; %axis([0 2.3e7 0 1]);
plot(current_BW(2:end), accum_U(2:end),'*');