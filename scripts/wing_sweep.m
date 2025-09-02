clc;
clear all;
close all;

AB = 170;    # Distance from wing to servo anchors
BC = 8;      # Length of servo arm
CD = 30;     # Length of intermediate linkage
AD = 171.4;  # Distance from wing pivot to linkage attachment point

function angle_rad = get_angle_cos_law(a, b, opp)
  top = a*a + b*b - opp*opp;
  bottom = 2 * a * b;
  angle = acos(top/bottom);
endfunction

number_of_steps = 180;
start_servo_angle = deg2rad(0);
x = linspace(start_servo_angle, start_servo_angle + 2 * pi, number_of_steps);
wing_angle = zeros(1, number_of_steps);

servo_increment = x(2) - x(1);

for i = linspace(1, number_of_steps, number_of_steps)
  servo_angle = x(i);
  cx = AB + BC * cos(servo_angle);
  cy = BC * sin(servo_angle);

  AC = sqrt(cx*cx + cy*cy);

  angle_cab = get_angle_cos_law(AC, AB, BC);
  angle_dab = get_angle_cos_law(AC, AD, CD);

  angle_ca_horizon = atan(cy / cx);

  wing_angle(i) = angle_ca_horizon + angle_cab + angle_dab;

  if i > 1
    wing_angle_gradient(i-1) = (wing_angle(i) - wing_angle(i-1)) / servo_increment;
  endif
endfor

wing_angle_gradient(number_of_steps) = wing_angle_gradient(number_of_steps-1);

x = rad2deg(x);
wing_angle = rad2deg(wing_angle);

figure('name', 'output_angle_plot');
plot(x, wing_angle);
xlim([start_servo_angle, start_servo_angle + 360]);
xticks(start_servo_angle + [0, 90, 180, 270, 360]);
title('Wing Angle with Servo Angle');
display('output_angle_plot');

[max_wing_angle, max_ind] = max(wing_angle);
[min_wing_angle, min_ind] = min(wing_angle);

max_servo_angle = x(max_ind);
min_servo_angle = x(min_ind);

printf("Max wing deflection of %.3f deg at servo angle of %.0f deg\n", max_wing_angle, max_servo_angle);
printf("Min wing deflection of %.3f deg at servo angle of %.0f deg\n", min_wing_angle, min_servo_angle);

delta = max_wing_angle - min_wing_angle;
servo_center = (max_servo_angle + min_servo_angle) / 2;

printf("Wing range of motion is %.3f degrees. Center servo at %.0f deg\n", delta, servo_center);

figure('name', 'linearity_of_output_to_input');
plot(x, wing_angle_gradient);
xlim([0, 360]);
xticks([0, 90, 180, 270, 360]);
title('Linearity of Wing Angle to Servo Angle');
display('linearity_of_output_to_input');

