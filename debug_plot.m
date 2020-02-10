clear all;

data = csvread('matlab_data.csv');
a_data = csvread('axis_data.csv');

figure()
hold off
joint(1) = 0;
joint_a(1) = 0;
a=1;
for i=1:1;
joint_counter = 1;
  
  for j=1:3:length(data(1,:))
    joint(joint_counter,1) = data(i,j);
    joint(joint_counter,2) = data(i,j+1);
    joint(joint_counter,3) = data(i,j+2);

    %for a=1:9:length(a_data(1,:))
      joint_a(joint_counter,1) = a_data(i,a);
      joint_a(joint_counter,2) = a_data(i,a+1);
      joint_a(joint_counter,3) = a_data(i,a+2);
      joint_a(joint_counter,4) = a_data(i,a+3);
      joint_a(joint_counter,5) = a_data(i,a+4);
      joint_a(joint_counter,6) = a_data(i,a+5);
      joint_a(joint_counter,7) = a_data(i,a+6);
      joint_a(joint_counter,8) = a_data(i,a+7);
      joint_a(joint_counter,9) = a_data(i,a+8);
      
      test_mat = [[joint_a(joint_counter, 1) joint_a(joint_counter, 2) joint_a(joint_counter, 3)]
                  [joint_a(joint_counter, 4) joint_a(joint_counter, 5) joint_a(joint_counter, 6)]
                  [joint_a(joint_counter, 7) joint_a(joint_counter, 8) joint_a(joint_counter, 9)]];
    disp('Testing')
    test_mat*test_mat'
    det(test_mat)
    
    %endfor
    a+=9;
    
    %plot3(joint(joint_counter,1), joint(joint_counter,2), joint(joint_counter,3), 'r+', 'linewidth',3)

%figure()    
        hold on
    plot_basis(joint_a(joint_counter, :), joint(joint_counter, :));
    
    
    joint_counter += 1;  
  endfor
  axis equal
  hold off
%pause(100)
  pause(0.03333)  
endfor


