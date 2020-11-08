% script to draw figures

load('icra19_sim1_ours.mat');
load('icra19_sim1_initmst.mat');
load('icra19_sim1_initcon.mat');

% plot the min inter-robot distance profile
lineStyles = linspecer(10);
idx_plo = 1:1500;
font_size = 25;
figure;  % plot RMS error first
plot(idx_plo,min_bot_dist_hist,'Color',lineStyles(10,:),'LineWidth',2,'LineStyle','-');
% hold on;
% plot(idx_plo,rms_bb(2,:),'Color',lineStyles(9,:),'LineWidth',2,'LineStyle','-');
hold on;
plot(idx_plo,min_bot_dist_hist_initmst,'Color',lineStyles(8,:),'LineWidth',2,'LineStyle','--');
hold on;
plot(idx_plo,min_bot_dist_hist_initcon,'Color',lineStyles(2,:),'LineWidth',2,'LineStyle','--');
set(gca,'LineWidth',2,'fontsize',font_size,'fontname','Times','FontWeight','Normal');
legend('Distributed Dynamic MCCST','Preserve Initial MST','Preserve Initial Connectivity Graph');
xlabel('Time Steps','FontName','Times New Roman','FontSize',font_size);
ylabel('Minimum Inter-Robot Distance (m)','FontName','Times New Roman','FontSize',font_size);
set(gca,'linewidth',2,'fontsize',font_size,'fontname','Times');


figure;  % plot RMS error first
plot(idx_plo,conn_bot_hist,'Color',lineStyles(10,:),'LineWidth',2,'LineStyle','-');
% hold on;
% plot(idx_plo,rms_bb(2,:),'Color',lineStyles(9,:),'LineWidth',2,'LineStyle','-');
hold on;
plot(idx_plo,conn_bot_hist_initmst,'Color',lineStyles(8,:),'LineWidth',2,'LineStyle','--');
hold on;
plot(idx_plo,conn_bot_hist_initcon,'Color',lineStyles(2,:),'LineWidth',2,'LineStyle','--');
set(gca,'LineWidth',2,'fontsize',font_size,'fontname','Times','FontWeight','Normal');
legend('Distributed Dynamic MCCST','Preserve Initial MST','Preserve Initial Connectivity Graph');
xlabel('Time Steps','FontName','Times New Roman','FontSize',font_size);
ylabel('Graph Connectivity - \lambda_2','FontName','Times New Roman','FontSize',font_size);
ylim([0 0.6])
set(gca,'linewidth',2,'fontsize',font_size,'fontname','Times');

figure;  % plot RMS error first
plot(idx_plo,pertub_bot_hist,'Color',lineStyles(10,:),'LineWidth',2,'LineStyle','-');
% hold on;
% plot(idx_plo,rms_bb(2,:),'Color',lineStyles(9,:),'LineWidth',2,'LineStyle','-');
hold on;
plot(idx_plo,pertub_bot_hist_initmst,'Color',lineStyles(8,:),'LineWidth',2,'LineStyle','--');
hold on;
plot(idx_plo,pertub_bot_hist_initcon,'Color',lineStyles(2,:),'LineWidth',2,'LineStyle','--');
set(gca,'LineWidth',2,'fontsize',font_size,'fontname','Times','FontWeight','Normal');
legend('Distributed Dynamic MCCST','Preserve Initial MST','Preserve Initial Connectivity Graph');
xlabel('Time Steps','FontName','Times New Roman','FontSize',font_size);
ylabel('Average Control Pertubance','FontName','Times New Roman','FontSize',font_size,'Interpreter','latex');
ylim([0 0.01]);
set(gca,'linewidth',2,'fontsize',font_size,'fontname','Times');



figure;  % plot RMS error first
plot(idx_plo,perf_bot_hist,'Color',lineStyles(10,:),'LineWidth',2,'LineStyle','-');
% hold on;
% plot(idx_plo,rms_bb(2,:),'Color',lineStyles(9,:),'LineWidth',2,'LineStyle','-');
hold on;
plot(idx_plo,perf_bot_hist_initmst,'Color',lineStyles(8,:),'LineWidth',2,'LineStyle','--');
hold on;
plot(idx_plo,perf_bot_hist_initcon,'Color',lineStyles(2,:),'LineWidth',2,'LineStyle','--');
set(gca,'LineWidth',2,'fontsize',font_size,'fontname','Times','FontWeight','Normal');
legend('Distributed Dynamic MCCST','Preserve Initial MST','Preserve Initial Connectivity Graph');
xlabel('Time Steps','FontName','Times New Roman','FontSize',font_size);
ylabel('Average Distance to Target (m)','FontName','Times New Roman','FontSize',font_size,'Interpreter','latex');
set(gca,'linewidth',2,'fontsize',font_size,'fontname','Times');
