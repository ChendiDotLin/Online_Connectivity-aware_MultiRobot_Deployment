close all;
clear;

load('numerical_res.mat');
lw = 1.5;
% Create figure 1 about loss
figure('PaperUnits','centimeters',...
    'PaperSize',[21.4285714285714 13.5714285714286],...
    'Color',[1 1 1]);

% Create axes
axes1 = axes;
hold(axes1,'on');
s = 10;
Ns = Ns(s:end);
losses = losses(:,s:end);
times = times(:,s:end);
plot(Ns,losses(1,:),'r-x','MarkerSize',10,'LineWidth',lw,'DisplayName','Adaptive Greedy');

plot(Ns,losses(2,:),'b-o','MarkerSize',10,'LineWidth',lw,'DisplayName','MIDACO');

plot(Ns,losses(3,:),'g-*','MarkerSize',10,'LineWidth',lw,'DisplayName','GA');

% semilogy(Ns,losses(1,:),'r-o','LineWidth',lw,'DisplayName','Adaptive Greedy');
% semilogy(Ns,losses(2,:),'b-o','LineWidth',lw,'DisplayName','MIDACO');
% semilogy(Ns,losses(3,:),'g-o','LineWidth',lw,'DisplayName','GA');
xlabel('Total number of robots')
ylabel('Remaining Utility (J_r)')


box(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'LineWidth',0.8);
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.563928571428571 0.742738095238095 0.341071428571429 0.183333333333333],...
    'FontSize',20,...
    'Color','none');


% Create figure 2 about Running time
figure('PaperUnits','centimeters',...
    'PaperSize',[21.4285714285714 13.5714285714286],...
    'Color',[1 1 1]);

% Create axes
axes2 = axes;
semilogy(Ns,times(1,:),'r-x','MarkerSize',10,'LineWidth',lw,'DisplayName','Adaptive Greedy');
hold(axes2,'on');

semilogy(Ns,times(2,:),'b-o','MarkerSize',10,'LineWidth',lw,'DisplayName','MIDACO');
semilogy(Ns,times(3,:),'g-*','MarkerSize',10,'LineWidth',lw,'DisplayName','GA');
% plot(Ns,times(1,:),'r-o','LineWidth',lw,'DisplayName','Adaptive Greedy');
% plot(Ns,times(2,:),'b-o','LineWidth',lw,'DisplayName','MIDACO');
% plot(Ns,times(3,:),'g-o','LineWidth',lw,'DisplayName','GA');
xlabel('Total number of robots')
ylabel('Running time (seconds) in log scale')


box(axes2,'on');
% Set the remaining axes properties
set(axes2,'FontSize',22,'LineWidth',0.8);
% Create legend
legend2 = legend(axes2,'show');
set(legend2,...
    'Position',[0.563928571428571 0.742738095238095 0.341071428571429 0.183333333333333],...
    'FontSize',20,...
    'Color','none');