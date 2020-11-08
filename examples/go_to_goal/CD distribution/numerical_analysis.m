close all; clc; clear all;
file_num = 32;

% L = zeros(1,3500);
L = [];
L_max = zeros(1,3500);
L_min = 1000*ones(1,3500);

dx = [];
dx_max = zeros(1,3500);
dx_min = 1000*ones(1,3500);
for i = 1:file_num
    file_name = strcat(strcat('res',num2str(i)),'.mat');
    load(file_name);
    %    L = L + L_vals;
    L = [L;L_vals];
    L_max = max(L_max,L_vals);
    L_min = min(L_min,L_vals);
    dxi = zeros(1,timer_count);
    for j = 1:timer_count
        dxi(j) = norm(mean(dxi_vals{j}));
    end
    dx = [dx;dxi];
    dx_max = max(dx_max,dxi);
    dx_min = min(dx_min,dxi);
end
% L = L/file_num;

figure
% plot([1:1:timer_count],L,'LineWidth',2);
% errorbar([1:1:timer_count],mean(L),std(L),'LineWidth',2);
shadedErrorBar([1:1:timer_count], mean(L),std(L), 'lineprops',{'-b', 'LineWidth', 2});
% hold on
% plot([1:1:timer_count],L(1,:),'LineWidth',2);
ylim([0,200]);
set(gca,'FontSize',15)
xlabel('Time Step','FontSize',20);
ylabel('Weighted Remaining Unfulfillment','FontSize',20)
print(gcf,'remaining.png','-dpng','-r300');

figure
% plot([1:1:timer_count],L,'LineWidth',2);
% errorbar([1:1:timer_count],mean(L),std(L),'LineWidth',2);
shadedErrorBar([1:1:timer_count], mean(dx),std(dx), 'lineprops',{'-b', 'LineWidth', 2});
% hold on
% plot([1:1:timer_count],L(1,:),'LineWidth',2);
% ylim([0,200]);
set(gca,'FontSize',15)
xlabel('Time Step','FontSize',20);
ylabel('Average Speed','FontSize',20)
print(gcf,'speed.png','-dpng','-r300');
