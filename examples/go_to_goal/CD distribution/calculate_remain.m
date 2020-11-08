function [remain,remain_no_value,explored,found] = calculate_remain(N,nt,x,targets,require,c,range,assigned,value,explored)
    % here we calculate the remaining requirements (modulize this part)
    % calculated from target pose and pose
    found = 0;
    num_cap = size(require,1);
    remain = zeros(1,N);

    remain_no_value = zeros(num_cap,nt);
    remain_sum = zeros(num_cap,1);
    for i = 1:nt
    dist_vec = vecnorm(x(1:2, :) - targets(:,i));

    remain_no_value(:,i) = max(0,require(:,i) - sum(c.*(dist_vec<range),2));

    % if there is a sudden change, there will be a convergence issue
    % so here we can try a sigmoid function to smooth it

    % if touches, explored
    if(explored(i)==0 && sum(dist_vec<range)~=0)
        explored(i) = 1;
        found = i;
    end
    sigmoid_remain = max(0,require(:,i) - sum(c.*(logsig(-(dist_vec/(range-0.05)*40-45))),2)); % we need to sigmoid the remaining function
    remain(:,assigned==i) = sum((c(:,assigned==i).*sigmoid_remain)*value(i),1);
    remain_sum  = remain_sum + sigmoid_remain*value(i);
    sig = logsig(-(dist_vec/(range-0.05)*40-45));


    
    

    end

    remain = remain/(max(remain)+0.001)*10;
    
    % add 1 so to avoid lack of rank in qp
    remain = remain+1;


