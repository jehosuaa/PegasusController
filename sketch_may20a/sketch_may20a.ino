float angulos(X, Y, Z, preQ1, preQ2, preQ3, preQ4){
  phi = preQ2 + preQ3 + preQ4;
  float the1 = th1(X, Y);
  x1 = X - l3* sin(phi)* cos(theta1);
  y1 = Y - l3* sin(phi)* sin(theta1); 
  z1 = Z - l3* cos(phi);
  k = sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2));
  float the3 = th3(k, l1, l2);
  float the2 = th2(the3, k, l2, z1);
  return the1, the2, the3;
}

void inversa(xa, ya, za, xb, yb, zb){
  a1, a2, a3= angulos(xa, ya, za);
  b1, b2, b3= angulos(xb, yb, zb);
  for (int t = 0, t <= 10, i++){
    x = (t/10)*(xb - xa) + xa;
    y = (t/10)*(yb - ya) + ya;
    z = (t/10)*(zb - za) + za;
    c1, c2, c3= angulos(x, y, z);
    
  }
}

void inputs(){
  
}

float th1(px, py){
  float theta1 = atan2(-px, py);
  return theta1;
}

float th2(theta3, k, L2, z1){
  float theta2 = (pi/2)-asin(L2*sin(theta3)/k)-asin(z1/k);
  return theta2;
}

float th3(k, L1, L2){
  float theta3 = acos((pow(k, 2)- pow(L1,2)- pow(L2, 2))/(2*L1*L2));
  return theta3;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  }
}
