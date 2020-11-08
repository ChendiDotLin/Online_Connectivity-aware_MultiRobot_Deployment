% mvee_algo_proc

load('darpa_mvee_data2.mat');
mean_num_messsage_set = [];
mean_num_core_set = [];
mean_area_lowner_set = [];
mean_area_core_set = [];
mean_area_rate_set = [];

valid_flag_set = ones(5,10);

for ijk = 1:num_bot_stack
mean_num_message_set(ijk) = mean(num_message_set(ijk,find(valid_flag_set(ijk,:))));
mean_num_core_set(ijk) = mean(num_core_set(ijk,find(valid_flag_set(ijk,:))));
mean_area_lowner_set(ijk) = mean(area_lowner_set(ijk,find(valid_flag_set(ijk,:))));
mean_area_core_set(ijk) = mean(area_core_set(ijk,find(valid_flag_set(ijk,:))));
mean_area_rate_set(ijk) = mean(area_rate_set(ijk,find(valid_flag_set(ijk,:))));
end