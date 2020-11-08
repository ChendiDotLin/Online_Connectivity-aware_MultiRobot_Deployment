% L_allocation = zeros(20,1400);
% L_original = zeros(20,1400);
% 
% for (i = 1:20)
%     close all;
%     try
%     L_allocation(i,:) = task_reallocation_2targets_go_to_point_mst(true);
%     catch 
% %     L_allocation(i,:) = task_reallocation_2targets_go_to_point_mst(true);
% 
%     end
%     close all;
%     try
%     L_original(i,:) = task_reallocation_2targets_go_to_point_mst(false);
% 
%     catch
% %     L_allocation(i,:) = task_reallocation_2targets_go_to_point_mst(true);
%     end
% end

% errorbar(mean(L_allocation),std(L_allocation),mean(L_original),std(L_original))

% save(strcat(cur_folder,'/numerical_results/first_numerical_res_2targets'),'L_allocation','L_original');

plot([1:1400],mean(L_allocation),'r','LineWidth',5)
hold on
plot([1:1400],mean(L_original),'b','LineWidth',5)    
hold off
legend("with redistribution","without redistribution",'FontSize',40)
xlabel("time step",'FontSize',30)
ylabel("remained requirement weighted by importance",'FontSize',30)
set(gca,'FontSize',30)
set(gcf,'unit','norm','position',[0 0 1 1])

