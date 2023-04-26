global mu_F;
global mu_S;
global r_F;
global r_S;
global mg;
mu_F = 0.52;
mu_S = 0.1833;
r_F = 0.015;
r_S = 0.04;
mg = 0.85;


mu_F = 0.52;
mu_S = 0.1833;
r_F = 0.015;
r_S = 0.04;
mg = 0.624;


f = figure;
set(gcf, 'Position',  [0, 0, 600, 760])
global ax;
ax= axes('Parent',f,'position',[0.13 0.55  0.77 0.40]);

b_mg = uicontrol('Parent',f,'Style','slider','Position',[81,24,419,20],...
              'value',mg, 'min',0, 'max',1);
b_muf = uicontrol('Parent',f,'Style','slider','Position',[81,84,419,20],...
              'value',mu_F, 'min',0, 'max',2);
b_mus = uicontrol('Parent',f,'Style','slider','Position',[81,144,419,20],...
              'value',mu_S, 'min',0, 'max',0.5);
b_rs = uicontrol('Parent',f,'Style','slider','Position',[81,204,419,20],...
              'value',r_S, 'min',0, 'max',0.05);
b_rf = uicontrol('Parent',f,'Style','slider','Position',[81,264,419,20],...
              'value',r_F, 'min',0, 'max',0.02);
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Position',[50,24,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl2 = uicontrol('Parent',f,'Style','text','Position',[500,24,23,23],...
                'String','1','BackgroundColor',bgcolor);
bl3 = uicontrol('Parent',f,'Style','text','Position',[240,0,100,23],...
                'String','mg','BackgroundColor',bgcolor);

bl4 = uicontrol('Parent',f,'Style','text','Position',[50,84,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl5 = uicontrol('Parent',f,'Style','text','Position',[500,84,23,23],...
                'String','2','BackgroundColor',bgcolor);
bl6 = uicontrol('Parent',f,'Style','text','Position',[240,60,100,23],...
                'String','mu_F','BackgroundColor',bgcolor);

bl7 = uicontrol('Parent',f,'Style','text','Position',[50,144,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl8 = uicontrol('Parent',f,'Style','text','Position',[500,144,23,23],...
                'String','0.5','BackgroundColor',bgcolor);
bl9 = uicontrol('Parent',f,'Style','text','Position',[240,120,100,23],...
                'String','mu_S','BackgroundColor',bgcolor);

bl10 = uicontrol('Parent',f,'Style','text','Position',[50,204,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl11 = uicontrol('Parent',f,'Style','text','Position',[500,204,23,23],...
                'String','0.05','BackgroundColor',bgcolor);
bl12 = uicontrol('Parent',f,'Style','text','Position',[240,180,100,23],...
                'String','r_S','BackgroundColor',bgcolor);

bl13 = uicontrol('Parent',f,'Style','text','Position',[50,264,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl14 = uicontrol('Parent',f,'Style','text','Position',[500,264,23,23],...
                'String','0.02','BackgroundColor',bgcolor);
bl15 = uicontrol('Parent',f,'Style','text','Position',[240,240,100,23],...
                'String','r_F','BackgroundColor',bgcolor);

b_mg.Callback = @(es,ed) set_mg(es.Value);
b_muf.Callback = @(es,ed) set_muf(es.Value);
b_mus.Callback = @(es,ed) set_mus(es.Value);
b_rf.Callback = @(es,ed) set_rf(es.Value);
b_rs.Callback = @(es,ed) set_rs(es.Value);

plot_DLS(mu_F, mu_S, r_F, r_S, mg);
function set_mg(data)    
    global mu_F;
    global mu_S;
    global r_F;
    global r_S;
    global mg;
    mg = data;
    plot_DLS(mu_F, mu_S, r_F, r_S, mg);
end

function set_muf(data)    
    global mu_F;
    global mu_S;
    global r_F;
    global r_S;
    global mg;
    mu_F = data;
    plot_DLS(mu_F, mu_S, r_F, r_S, mg);
end

function set_mus(data)    
    global mu_F;
    global mu_S;
    global r_F;
    global r_S;
    global mg;
    mu_S = data;
    plot_DLS(mu_F, mu_S, r_F, r_S, mg);
end

function set_rf(data)    
    global mu_F;
    global mu_S;
    global r_F;
    global r_S;
    global mg;
    r_F = data;
    plot_DLS(mu_F, mu_S, r_F, r_S, mg);
end

function set_rs(data)    
    global mu_F;
    global mu_S;
    global r_F;
    global r_S;
    global mg;
    r_S = data;
    plot_DLS(mu_F, mu_S, r_F, r_S, mg);
end




function plot_DLS(mu_F, mu_S, r_F, r_S, mg)
    global ax;
    cla(ax) 


    [x,y] = meshgrid(-10 * mu_S:0.05 * mu_S:10 * mu_S,-10*mu_S * 0.6 * r_S:0.05*mu_S * 0.6 * r_S:10*mu_S * 0.6 * r_S); 
    Z1 = sqrt((x / mu_S).^2+(y / (mu_S * 0.6 * r_S)).^2);
    surf(x,y,Z1, 'FaceColor','g', 'FaceAlpha', 0.5, 'EdgeColor','none');
    hold on
    
    % limit surface of finger
    Z2 = mg + sqrt((x / mu_S).^2 / (mu_F/mu_S)^2+(y / (mu_S * 0.6 * r_S)).^2 / ((mu_F*r_F/mu_S/r_S))^2);
    surf(x,y,Z2, 'FaceColor','r', 'FaceAlpha', 0.5, 'EdgeColor','none')
    xlabel('F'), ylabel('T'), zlabel('F_N')
    title('mg='+string(mg)+', mu_F='+string(mu_F) +', mu_S='+string(mu_S)+', r_F='+string(r_F)+', r_S='+string(r_S),'Units', 'normalized', 'Position', [0.5, -0.38, 0])
    axis vis3d
end
 