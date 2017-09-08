clear all; close all; clc;
BW = 30; I = 16; Q = 5; U = abs(rand(I, Q)); C = abs(log(rand(I,Q))); state = floor(mod(Q*abs(randn(1,I)),Q));
U = sort(U,2); C = sort(C,2);
%function [stateOut totalU] = TBRS_AR(BW, I, Q, U, C, state)
%% Tile-based rate selection algorithm (Convex Hull)
% parameters
x = zeros(I,Q); % selection parameter
u = zeros(I,Q); % marginal utility

% Current state
stateIn = state;

% initial lambda
lamb = zeros(I,Q); 
for ii=1:I
    if state(ii)==0
        for qq=1:Q
            lamb(ii,qq) = U(ii,qq)/C(ii,qq);
            marU(ii,qq) = U(ii,qq);
            marC(ii,qq) = C(ii,qq); 
        end
    else
        for qq=1:Q
            %if(qq<state(ii)), U(ii,qq) = U(ii,state(ii)); end
            lamb(ii,qq) = (U(ii,qq)-U(ii,state(ii)))/C(ii,qq); 
            marU(ii,qq) = U(ii,qq)-U(ii,state(ii));
            marC(ii,qq) = C(ii,qq); 
        end 
    end
end 

currentBW = 0; accumU = 0; cc = 1; rank = zeros(I,Q); rankvector = zeros(1,I*Q); ranking = 0;
% Iteration for bit allocation
while currentBW(cc) <= BW
    ranking = ranking + 1;
    % Intitial value
    Uvector = reshape(marU,[1 I*Q]);
    Cvector = reshape(marC,[1 I*Q]);
    xvector = reshape(x,[1 I*Q]);
    lamb_vector = reshape(lamb,[1 I*Q]);
    [lambSort(cc) lambIndex(cc)] = max(lamb_vector);
    rankvector(lambIndex(cc)) = ranking;

    currentBW(cc+1) = currentBW(cc) + Cvector(lambIndex(cc));
    accumU(cc+1) = accumU(cc) + Uvector(lambIndex(cc));
    xvector(lambIndex(cc)) = 1;

    x = reshape(xvector, [I Q]);
    if mod(lambIndex(cc),I) == 0
        state(I) = ceil(lambIndex(cc)/I);
        updateI = I;
    else
        state(mod(lambIndex(cc),I)) = ceil(lambIndex(cc)/I);
        updateI = mod(lambIndex(cc),I);
    end
    % if statement for break the while loop when Quality reach to max
    
    lamb(updateI,:) = zeros(1,Q);
    % update lambda
    for qq=state(updateI)+1:Q
        lamb(updateI,qq) = (U(updateI,qq)-U(updateI,state(updateI)))/(C(updateI,qq)-C(updateI,state(updateI))); 
        marU(updateI,qq) = U(updateI,qq)-U(updateI,state(updateI));
        marC(updateI,qq) = C(updateI,qq)-C(updateI,state(updateI)); 
    end 
    if(sum(sum(lamb))==0), break; end
    
    cc = cc + 1;
end
stateOut = state;
rank = reshape(rankvector, [I Q]);
figure(1);
for ff=1:I
    st = string;
    subplot(4,4,ff); hold; 
    plot(C(ff,:),U(ff,:),'*');
    if stateIn(ff)>0
        plot(C(ff,stateIn(ff)),U(ff,stateIn(ff)),'ko');
        plot([0 ceil(max(max(C)))],[U(ff,stateIn(ff)) U(ff,stateIn(ff))],'--k');
    else
        plot(0,0,'ko');
        plot([0 ceil(max(max(C)))],[0 0],'--k');
    end
    [a b] = sort(U(ff,:).*x(ff,:)); c = sum(a>0);
    if c>0,
        xt = [0 C(ff,b(end-c+1:end)).*x(ff,b(end-c+1:end))];
        if stateIn(ff)>0
            yt = [U(ff,stateIn(ff)) U(ff,b(end-c+1:end)).*x(ff,b(end-c+1:end))];
        else
            yt = [0 U(ff,b(end-c+1:end)).*x(ff,b(end-c+1:end))];
        end
        plot(xt,yt,'r-'); 
        newb = b(end-c+1:end);
        for ss=1:length(xt)-1, 
            st{ss} = num2str(rank(ff,newb(ss)));
            text(xt(ss+1), yt(ss+1)+0.05, st{ss});
        end
    end
    axis([0 ceil(max(max(C))) 0 ceil(max(max(U)))]);
    legend('data','initial state','initial utility','convex hull')
    ylabel('utility')
    xlabel('cost (bits)')
    grid on;
end

%end