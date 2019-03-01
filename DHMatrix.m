function R = DHMatrix(theta, d, a, alpha)
%D-H变换（Bruno书籍）

%先沿着Z轴平移di再旋转theta角度，再沿着X轴平移ai再旋转alpha角度
%syms theta d a alpha;

R = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
	sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
	0, sin(alpha), cos(alpha), d;
	0, 0, 0, 1];