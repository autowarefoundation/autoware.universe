% Assume output from LevelEllipsoid has been read in
doinside=1;
red=[179,27,27]/255;
white=[1,1,1];
black=[0,0,0];
blue=[0,19,56]/100;
green=[9,45,27]/100;
gray=[0.9,0.9,0.9];
thick=1;
pltsize=[5.9 4.12];
set(gcf,'Units','pixels');
set(gcf,'Position',50+150*[0 0 pltsize]);
set(gcf,'Units','normalized');
hold off;
if ~doinside
  fill([p{1}(:,1);0],[p{1}(:,2);0], gray, 'EdgeColor', 'none');
  hold on
end
nq=size(q,2);
for i=1:nq;
  plot(q{i}(:,1),q{i}(:,2), 'Color', green);
  if i == 1, hold on; end
end
np=size(p,2);
for i=1:np
  color = blue;
  if i == 1
    plot(p{i}(:,1),p{i}(:,2), 'Color', red, 'LineWidth', thick);
  else
    plot(p{i}(:,1),p{i}(:,2), 'Color', blue);
  end
end
if doinside
  nq=size(qa,2);
  for i=1:nq;
    plot(qa{i}(:,1),qa{i}(:,2), 'Color', green);
  end
  np=size(pa,2);
  for i=1:np
    plot(pa{i}(:,1),pa{i}(:,2), 'Color', blue);
  end
  plot([0, 0.6], [0, 0], 'Color', black, 'LineWidth', thick);
end
hold off
xlabel('R'); ylabel('Z');
axis equal;
axis([0,xmax,0,ymax]);
set(gcf,'PaperOrientation', 'landscape');
set(gcf,'PaperSize',pltsize);
set(gcf,'PaperPosition',[0 0 pltsize]);
set(gca, 'XTick',[0:3]);
set(gca, 'YTick',[0:2]);
set(gca, 'LooseInset',[0.07 0.09 0.03 0.02]);
ylabelh=get(gca,'ylabel');
set(ylabelh,'rotation',0);
set(ylabelh,'Position', get(ylabelh, 'Position') + [-0.1 0.2 0]);
print('-dsvg', ['normal-gravity-potential-', num2str(doinside), '.svg']);
