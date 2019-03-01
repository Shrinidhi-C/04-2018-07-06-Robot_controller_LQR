function R = DHMatrix(theta, d, a, alpha)
%D-H�任��Bruno�鼮��

%������Z��ƽ��di����תtheta�Ƕȣ�������X��ƽ��ai����תalpha�Ƕ�
%syms theta d a alpha;

R = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
	sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
	0, sin(alpha), cos(alpha), d;
	0, 0, 0, 1];