load wind
wind_speed = sqrt(u.^2 + v.^2 + w.^2);
figure
p = patch(isosurface(x,y,z,wind_speed,35));
isonormals(x,y,z,wind_speed,p)
p.FaceColor = [0.75,0.25,0.25];
p.EdgeColor = [0.6,0.4,0.4];

[f,vt] = reducepatch(isosurface(x,y,z,wind_speed,45),0.05); 
daspect([1,1,1]);
hcone = coneplot(x,y,z,u,v,w,vt(:,1),vt(:,2),vt(:,3),2);
hcone.FaceColor = 'blue';
hcone.EdgeColor = 'none';

camproj perspective;
camva(25);
hlight = camlight('headlight'); 
p.AmbientStrength = 1;
p.SpecularStrength = 1;
p.DiffuseStrength = 1;
hcone.SpecularStrength = 1;
set(gcf,'Color','k');
set(gca,'Color',[0,0,0.25]);
lighting gouraud;
hsline = streamline(x,y,z,u,v,w,80,30,11);
xd = hsline.XData;
yd = hsline.YData;
zd = hsline.ZData; 
delete(hsline);

for i=1:length(xd)-5
   campos([xd(i),yd(i),zd(i)])
   camtarget([xd(i+5)+min(xd)/500,yd(i),zd(i)])
   camlight(hlight,'headlight')
   drawnow
end

surf(peaks)
axis vis3d
t = 0:pi/20:4*pi;
dx = sin(t)./40;
dy = cos(t)./40;
for i = 1:length(t)
    camdolly(dx(i),dy(i),0)
    drawnow
end

