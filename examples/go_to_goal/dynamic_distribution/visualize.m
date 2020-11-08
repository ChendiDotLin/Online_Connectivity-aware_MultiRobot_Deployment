l = 17;
try
    delete(barplot111);
    delete(barplot121);
    delete(barplot112);
    delete(barplot122);
    delete(barplot131);
    delete(barplot132);
    delete(barplot141);
    delete(barplot142);
    delete(h111);
    delete(h121);
    delete(h131);
    delete(h141);
catch
end

try
    delete(barplot211);
    delete(barplot221);
    delete(barplot212);
    delete(barplot222);
    delete(barplot231);
    delete(barplot232);
    delete(barplot241);
    delete(barplot242);
    delete(h211);
    delete(h221);
    delete(h231);
    delete(h241);
catch
end

try
    delete(barplot311);
    delete(barplot321);
    delete(barplot312);
    delete(barplot322);
    delete(barplot331);
    delete(barplot332);
    delete(barplot341);
    delete(barplot342);
    delete(h311);
    delete(h321);
    delete(h331);
    delete(h341);
    
catch
end
% first two are the remain, the third is the requirement
barplot111 = plot([targets(1,1)-0.65,targets(1,1)-0.65],[targets(2,1),targets(2,1)+remain_no_value(1,1)/50],'g','LineWidth',l);
barplot121 = plot([targets(1,1)-0.55,targets(1,1)-0.55],[targets(2,1),targets(2,1)+remain_no_value(2,1)/50],'m','LineWidth',l);
barplot112 = plot([targets(1,1)-0.65,targets(1,1)-0.65],[targets(2,1),targets(2,1)+require(1,1)/50],'g','LineWidth',l);
barplot112.Color(4) = 0.2;
barplot122 = plot([targets(1,1)-0.55,targets(1,1)-0.55],[targets(2,1),targets(2,1)+require(2,1)/50],'m','LineWidth',l);
barplot122.Color(4) = 0.2;
barplot131 = plot([targets(1,1)-0.45,targets(1,1)-0.45],[targets(2,1),targets(2,1)+remain_no_value(3,1)/50],'c','LineWidth',l);
barplot141 = plot([targets(1,1)-0.35,targets(1,1)-0.35],[targets(2,1),targets(2,1)+remain_no_value(4,1)/50],'Color',[1 0.5 0.5],'LineWidth',l);
barplot132 = plot([targets(1,1)-0.45,targets(1,1)-0.45],[targets(2,1),targets(2,1)+require(3,1)/50],'c','LineWidth',l);
barplot132.Color(4) = 0.2;
barplot142 = plot([targets(1,1)-0.35,targets(1,1)-0.35],[targets(2,1),targets(2,1)+require(4,1)/50],'Color',[1 0.5 0.5],'LineWidth',l);
barplot142.Color(4) = 0.2;
tt111 = sprintf("%d/%d",remain_no_value(1,1),require(1,1));
tt121 = sprintf("%d/%d",remain_no_value(2,1),require(2,1));
tt131 = sprintf("%d/%d",remain_no_value(3,1),require(3,1));
tt141 = sprintf("%d/%d",remain_no_value(4,1),require(4,1));
h111 = text(targets(1,1)-0.7 ,targets(2,1)+require(1,1)/50+0.05,tt111,'FontSize',15);
if(explored(1)~=0)
h121 = text(targets(1,1)-0.6 ,targets(2,1)+require(2,1)/50+0.05,tt121,'FontSize',15);
h131 = text(targets(1,1)-0.5 ,targets(2,1)+require(3,1)/50+0.05,tt131,'FontSize',15);
h141 = text(targets(1,1)-0.4 ,targets(2,1)+require(4,1)/50+0.05,tt141,'FontSize',15);
end

barplot211 = plot([targets(1,2)+0.45,targets(1,2)+0.45],[targets(2,2)-0.2,targets(2,2)+remain_no_value(1,2)/50-0.2],'g','LineWidth',l);
barplot221 = plot([targets(1,2)+0.55,targets(1,2)+0.55],[targets(2,2)-0.2,targets(2,2)+remain_no_value(2,2)/50-0.2],'m','LineWidth',l);
barplot212 = plot([targets(1,2)+0.45,targets(1,2)+0.45],[targets(2,2)-0.2,targets(2,2)+require(1,2)/50-0.2],'g','LineWidth',l);
barplot212.Color(4) = 0.2;
barplot222 = plot([targets(1,2)+0.55,targets(1,2)+0.55],[targets(2,2)-0.2,targets(2,2)+require(2,2)/50-0.2],'m','LineWidth',l);
barplot222.Color(4) = 0.2;
barplot231 = plot([targets(1,2)+0.65,targets(1,2)+0.65],[targets(2,2)-0.2,targets(2,2)+remain_no_value(3,2)/50-0.2],'c','LineWidth',l);
barplot241 = plot([targets(1,2)+0.75,targets(1,2)+0.75],[targets(2,2)-0.2,targets(2,2)+remain_no_value(4,2)/50-0.2],'Color',[1 0.5 0.5],'LineWidth',l);
barplot232 = plot([targets(1,2)+0.65,targets(1,2)+0.65],[targets(2,2)-0.2,targets(2,2)+require(3,2)/50-0.2],'c','LineWidth',l);
barplot232.Color(4) = 0.2;
barplot242 = plot([targets(1,2)+0.75,targets(1,2)+0.75],[targets(2,2)-0.2,targets(2,2)+require(4,2)/50-0.2],'Color',[1 0.5 0.5],'LineWidth',l);
barplot242.Color(4) = 0.2;
tt211 = sprintf("%d/%d",remain_no_value(1,2),require(1,2));
tt221 = sprintf("%d/%d",remain_no_value(2,2),require(2,2));
tt231 = sprintf("%d/%d",remain_no_value(3,2),require(3,2));
tt241 = sprintf("%d/%d",remain_no_value(4,2),require(4,2));
h211 = text(targets(1,2)+0.4 ,targets(2,2)+require(1,2)/50-0.15,tt211,'FontSize',15);
if(explored(2)~=0)
h221 = text(targets(1,2)+0.5 ,targets(2,2)+require(2,2)/50-0.15,tt221,'FontSize',15);
h231 = text(targets(1,2)+0.6 ,targets(2,2)+require(3,2)/50-0.15,tt231,'FontSize',15);
h241 = text(targets(1,2)+0.7 ,targets(2,2)+require(4,2)/50-0.15,tt241,'FontSize',15);
end

if(nt==3)
    barplot311 = plot([targets(1,3)-0.15,targets(1,3)-0.15],[targets(2,3)-0.9,targets(2,3)+remain_no_value(1,3)/50-0.9],'g','LineWidth',l);
    barplot321 = plot([targets(1,3)-0.05,targets(1,3)-0.05],[targets(2,3)-0.9,targets(2,3)+remain_no_value(2,3)/50-0.9],'m','LineWidth',l);
    barplot312 = plot([targets(1,3)-0.15,targets(1,3)-0.15],[targets(2,3)-0.9,targets(2,3)+require(1,3)/50-0.9],'g','LineWidth',l);
    barplot312.Color(4) = 0.2;
    barplot322 = plot([targets(1,3)-0.05,targets(1,3)-0.05],[targets(2,3)-0.9,targets(2,3)+require(2,3)/50-0.9],'m','LineWidth',l);
    barplot322.Color(4) = 0.2;
    barplot331 = plot([targets(1,3)+0.05,targets(1,3)+0.05],[targets(2,3)-0.9,targets(2,3)+remain_no_value(3,3)/50-0.9],'c','LineWidth',l);
    barplot341 = plot([targets(1,3)+0.15,targets(1,3)+0.15],[targets(2,3)-0.9,targets(2,3)+remain_no_value(4,3)/50-0.9],'Color',[1 0.5 0.5],'LineWidth',l);
    barplot332 = plot([targets(1,3)+0.05,targets(1,3)+0.05],[targets(2,3)-0.9,targets(2,3)+require(3,3)/50-0.9],'c','LineWidth',l);
    barplot332.Color(4) = 0.2;
    barplot342 = plot([targets(1,3)+0.15,targets(1,3)+0.15],[targets(2,3)-0.9,targets(2,3)+require(4,3)/50-0.9],'Color',[1 0.5 0.5],'LineWidth',l);
    barplot342.Color(4) = 0.2;
    tt311 = sprintf("%d/%d",remain_no_value(1,3),require(1,3));
    tt321 = sprintf("%d/%d",remain_no_value(2,3),require(2,3));
    tt331 = sprintf("%d/%d",remain_no_value(3,3),require(3,3));
    tt341 = sprintf("%d/%d",remain_no_value(4,3),require(4,3));
    h311 = text(targets(1,3)-0.2 ,targets(2,3)+require(1,3)/50-0.85,tt311,'FontSize',15);
    if(explored(3)~=0)
    h321 = text(targets(1,3)-0.1 ,targets(2,3)+require(2,3)/50-0.85,tt321,'FontSize',15);
    h331 = text(targets(1,3)-0 ,targets(2,3)+require(3,3)/50-0.85,tt331,'FontSize',15);
    h341 = text(targets(1,3)+0.1 ,targets(2,3)+require(4,3)/50-0.85,tt341,'FontSize',15);
    end
end