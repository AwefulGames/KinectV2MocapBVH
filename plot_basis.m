function plot_basis(basis, pos)
  
  r = 10;
  
  
  
  
  % x axis
  plot3([pos(1), pos(1) + basis(1)*r], [pos(2), pos(2) + basis(2)*r] , [pos(3), pos(3) + basis(3)*r],'r');
  plot3([pos(1), pos(1) + basis(4)*r], [pos(2), pos(2) + basis(5)*r] , [pos(3), pos(3) + basis(6)*r],'m');
  plot3([pos(1), pos(1) + basis(7)*r], [pos(2), pos(2) + basis(8)*r] , [pos(3), pos(3) + basis(9)*r],'y');
  
  %plot3([pos (pos + basis(1:3)*r)],'r');
  %plot3([pos (pos + basis(4:6)*r)],'m');
  %plot3([pos (pos + basis(7:9)*r)],'y');
  
endfunction
