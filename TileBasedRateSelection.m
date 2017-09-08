clear all; close all; %clc;
%% Tile-based rate selection algorithm (Bell lab)
% parameters
I = 16; % number of tiles
Q = 4; % number of representations
x = zeros(I,Q); % selection parameter
U = zeros(I,Q); % marginal utility
P = ones(I,Q); % probability of view
pro = [0.7947 0.5774 0.4400 0.2576 0.7519 0.2287 0.2000 0.7673 0.6712 ...
    0.7152 0.6421 0.4190 0.3908 0.9161 0.3174 0.8145]; 
%pro = max([0.2*ones(I,1) rand(I,1)]')'; 
for ii=1:I, P(ii,:) = pro(ii)*P(ii,:); end
C = zeros(I,Q); % marginal cost
BW = 1e6; % bandwidth
BW2 = 27e6;

% Videos
Rv = [7.071e4 1.462e5 5.617e5 1.105e6;
    6.172e4 2.162e5 5.017e5 1.205e6;
    5.373e4 2.362e5 5.637e5 1.305e6;
    2.475e4 2.562e5 5.517e5 1.315e6;
    7.531e4 2.732e5 5.817e5 1.405e6;
    4.221e4 2.462e5 5.417e5 1.555e6;
    5.361e4 2.662e5 5.612e5 1.765e6;
    6.421e4 2.822e5 5.513e5 1.205e6;
    2.726e4 2.222e5 5.617e5 1.345e6;
    9.352e4 2.946e5 5.917e5 1.335e6;
    6.378e4 1.732e5 5.817e5 1.505e6;
    6.311e4 1.122e5 5.617e5 1.705e6;
    7.501e4 1.962e5 5.717e5 1.305e6;
    4.723e4 1.462e5 5.617e5 1.125e6;
    3.425e4 1.666e5 5.617e5 1.302e6;
    5.215e4 1.142e5 5.647e5 1.125e6];
%Rv = 1e6*rand(I,Q);
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
%a = I/log(max(current_BW)*sum(pro));
%fitU = min(alpha1).*log(min(pro)*beta1(1).*current_BW./tempA(1));
f=fit(log(current_BW(2:end))',accum_U(2:end)','poly2');
%tempU = f.p1*log(current_BW(2:end))+f.p2;
tempU = f.p1*log(current_BW(2:end)).^2+f.p2*log(current_BW(2:end))+f.p3;
%plot(current_BW(2:end),tempU,'r'); legend('data','fitted curve');
%plot(current_BW, fitU*8,'r');
plot(current_BW(2:end),tempU,'r'); legend('data','fitted curve');
figure(2); bar(sum(basex,2)); axis([0 17 0 4]); hold on; plot(pro,'r')
basel = sum(basex,2);

% New cost
for kk=1:Vn,
    for ll=1:Nv, 
        if ll<=basel(kk), C2(kk,ll) = 1e8;
        elseif ll==basel(kk)+1, C2(kk,ll) = Rv(kk,ll);
        else C2(kk,ll) = Rv(kk,ll) - Rv(kk,ll-1); end 
    end
end% marginal utility
uocr2 = U.*P./C2; % Utility over cost ratio
uocr_vector2 = reshape(uocr,[1 I*Q]);
[uocr_sort2 uocr_index2] = sort(uocr_vector2,'descend');
C2_vector = reshape(C2,[1 I*Q]);

% Enhancement group Iteration
for jj=ii:length(uocr_index),
    if current_BW(jj) <= BW+BW2, 
        current_BW(jj+1) = current_BW(jj) + C2_vector(uocr_index2(jj));
        accum_U(jj+1) = accum_U(jj) + U_vector(uocr_index2(jj))*P_vector(uocr_index2(jj));
        x_vector(uocr_index2(jj)) = 1;
        enhancex = reshape(x_vector, [I Q]);
    else break; end
end
enhanceU = sum(sum(enhancex.*U,2).*pro)
figure(3); grid on; xlabel('Available Rate'); ylabel('Utility'); hold on; 
%axis([0 2.3e7 0 1]);
plot(current_BW(ii+1:end), accum_U(ii+1:end),'*');
% a = I/log(max(current_BW)*sum(pro));
% fitU = min(alpha1).*log(min(pro)*beta1(1).*current_BW./tempA(1));
f2=fit(log(current_BW(ii+1:end))',accum_U(ii+1:end)','poly1');
tempU = f2.p1*log(current_BW(ii+1:end))+f2.p2;
plot(current_BW(ii+1:end),tempU,'r'); legend('data','fitted curve');
figure(4); bar(sum(enhancex,2)); axis([0 17 0 4]); hold on; plot(pro,'r')