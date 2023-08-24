noReal = 1;
sim_para = paras_sim;
sim_para.C = 10; sim_para.H = 5;
sim_para.beta0_h = db2lin(-10.0);
sim_para.Pmax_UAV = db2lin(30-30);
sim_para.n0 = db2lin(-174-30);
sim_para.B = 10e6;

conv_cur_All = cell(1,noReal);
post_UE_All = cell(1,noReal);

tic
for i = 1:noReal

    post_UE = positions(sim_para);
    post_UE_All{1,i} = post_UE;
       
    figid = 1;
    [t,f,P,p,b,q,eta,obj_cur,conv_cur,cv_curve] = nlnEH_3b(sim_para,post_UE);
    conv_cur_All{1,i} = conv_cur;

end
toc

figure(figid)
hold on;
plot(1:length(conv_cur),conv_cur,'b-^','linewidth',3.0,...
    'markers',12,'MarkerIndices',1:1:length(conv_cur));
hold off;
set(gca,'FontSize',25,'XLim',[1 length(conv_cur)]);
xlabel('Iteration Index'); 
ylabel('Total Energy Consumption (J)');
legend('E2FL')
box on;