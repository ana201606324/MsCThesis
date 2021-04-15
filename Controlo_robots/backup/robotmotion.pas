unit robotmotion;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, IniPropStorage, SdpoSerial, LCLIntf, LCLType,
  ComCtrls, channels, math, lNet, Unit1;

implementation

const

  // GotoXYTheta states
  Rotate = 0;
  Go_Forward = 1;
  De_Accel = 2;
  Final_Rot = 3;
  DeAccel_Final_Rot = 4;
  Stop = 5;

  W_NOM = 10;
  LIN_VEL_NOM = 3;
  LIN_VEL_DA = 0.5;
  GAIN_FWD = 100;
  GAIN_DA = 100;
  GAIN_DIST = 500;
  W_DA = 2;
  TOL_FINTHETA = 0.05;    // aprox. 3 deg.
  THETA_DA = 0.25;
  TOL_FINDIST = 0.01;
  DIST_DA = 0.05;
  MAX_ETF = 1;
  DIST_NEWPOSE = 0.02;
  THETA_NEWPOSE = 0.1;

  MAX_DIST2LINE = 0.3;
  NEAR_LINE = 0.25;

  // Followline states
  Follow_line = 0;
  De_Accel_Follow = 1;
  Goto_Near_Point = 2;
  Goto_XiYi = 3;
  Goto_XaYa = 4;
  Stop_Follow = 5;

  var
  goto_result:array[1...3] of double;
  followline_result:array[1...3] of double;
  v1, v2: double;
  linear_vel, angular_vel: double;
  state_Goto, state_Follow: integer;
  xr_e, yr_e, tr_e: double;
  position_robot:array[1...3, 1...3] of double;

procedure gotoXYTheta(i:integer; xf, yf, tf: double);
  var sign_dir, sign_dir_f, erro_dist, erro_theta, erro_theta_f: double;
  begin

    //obter coordenadas do robot i
    xr_e:= form1.pos_robot[i][0];
    yr_e:= form1.pos_robot[i][1];
    tr_e := form1.pos_robot[i][3];

    //Calc errors
    erro_theta := NormalizeAngle(tr_e - ATan2(yf - yr_e, xf - xr_e));
    erro_dist := sqrt(power(xr_e - xf,2) + power(yr_e -yf,2));
    erro_theta_f := NormalizeAngle(tf - tr_e);

    //Find fastest Rotate
    if erro_theta > 0 then begin
      sign_dir := -1;
    end else begin
      sign_dir := 1;
    end;

    if erro_theta_f > 0 then begin
      sign_dir_f := 1;
    end else begin
      sign_dir_f := -1;
    end;

    //Transitions
    case state_Goto of
      Rotate: begin
        if abs(erro_theta) < MAX_ETF then begin
          state_Goto := Go_Forward;
        end else if erro_dist < TOL_FINDIST then begin
          state_Goto := Final_Rot;
        end;
      end;

      Go_Forward: begin
        if erro_dist < DIST_DA then state_Goto := De_Accel;
      end;

      De_Accel: begin
        if erro_dist < TOL_FINDIST then state_Goto := Final_Rot;
      end;

      Final_Rot: begin
        if erro_theta_f < THETA_DA then begin
          state_Goto := DeAccel_Final_Rot;
        end else begin
          if erro_dist > DIST_NEWPOSE then state_Goto := Rotate;
        end;
      end;

      DeAccel_Final_Rot: begin
        if erro_theta_f < TOL_FINTHETA then begin
          state_Goto := Stop;
        end else begin
          if erro_dist > DIST_NEWPOSE then state_Goto := Rotate;
        end;
      end;

      Stop: begin
        if (abs(erro_theta_f) > THETA_NEWPOSE) or (erro_dist > DIST_NEWPOSE) then begin
          state_Goto := Rotate;
        end;
      end;
    end;

    //Outputs
    case state_Goto of
      Rotate : begin
        linear_vel := 0;
        angular_vel := sign_dir*W_NOM;
      end;

      Go_Forward : begin
        linear_vel := LIN_VEL_NOM;
        angular_vel := -GAIN_FWD*erro_theta;
      end;

      De_Accel : begin
        linear_vel := LIN_VEL_DA;
        angular_vel := -GAIN_DA*erro_theta;
      end;

      Final_Rot : begin
        linear_vel := 0;
        angular_vel := sign_dir_f*W_NOM;
      end;

      DeAccel_Final_Rot : begin
        linear_vel := 0;
        angular_vel := sign_dir_f*W_DA;
      end;

      Stop : begin
        linear_vel := 0;
        angular_vel := 0;
      end;
    end;

    // calculate v1 and v2 using linear_vel and angular_vel
    v1 := linear_vel - angular_vel*b/2;
    v2 := linear_vel + angular_vel*b/2;

    goto_result := (i, v1, v2);

end;

procedure FollowLine(i: integer; xf, yf, xi, yi: double);
var
  ux, uy, xn, yn,  dist2line, ang_line, xir, yir, xfr, yfr, xr_er, yr_er, xa, ya: double;
  sign_dir, erro_dist, erro_theta: double;
begin

  //obter coordenadas do robot i
    xr_e:= pos_robot[i][0];
    yr_e:= pos_robot[i][1];
    tr_e := pos_robot[i][3];

  // Distance to line
  ux := (xf - xi)/ sqrt(sqr(xf - xi) + sqr(yf - yi));
  uy := (yf - yi)/ sqrt(sqr(xf - xi) + sqr(yf - yi));
  dist2line := ((xr_e - xi)*uy + (yi - yr_e)*ux)/(sqr(ux) + sqr(uy));
  xn := xr_e + dist2line*(-uy);
  yn := yr_e + dist2line*ux;
  ang_line := ATan2(yf - yi, xf - xi);

  // Rotated points
  xir := xi*cos(ang_line) + yi*sin(ang_line);
  yir := -xi*sin(ang_line) + yi*cos(ang_line);
  xfr := xf*cos(ang_line) + yf*sin(ang_line);
  yfr := -xf*sin(ang_line) + yf*cos(ang_line);
  xr_er := xr_e*cos(ang_line) + yr_e*sin(ang_line);
  yr_er := -xr_e*sin(ang_line) + yr_e*cos(ang_line);

  xa := (xf - xi)*3/4 + xi;
  ya := (yf - yi)*3/4 + yi;

  SetRCValue(14,1, format('%.2g', [xr_er]));
  SetRCValue(15,1, format('%.2g', [xfr]));
  SetRCValue(16,1, format('%.2g', [xr_er - xfr]));

  // other errors
  erro_theta := NormalizeAngle(tr_e - ATan2(yf - yr_e, xf - xr_e));
  erro_dist := sqrt(power(xr_e - xf,2) + power(yr_e -yf,2));

  //Transitions
  case state_Follow of

    Follow_line: begin
      if erro_dist < DIST_DA then state_Follow := De_Accel_Follow;
      if abs(dist2line) > MAX_DIST2LINE then state_Follow := Goto_Near_Point;
      if (xr_er < xir) and (abs(dist2line) > MAX_DIST2LINE) then state_Follow := Goto_XiYi;
      if (xr_er > xfr) then state_Follow := Goto_XaYa;
    end;

    De_Accel_Follow: begin
      if erro_dist < TOL_FINDIST then state_Follow := Stop_Follow;
    end;

    Goto_Near_Point: begin
      if abs(dist2line) < NEAR_LINE then state_Follow := Follow_line;
    end;

    Goto_XiYi: begin
      if  abs(dist2line) < NEAR_LINE then state_Follow := Follow_line;
    end;

    Goto_XaYa: begin
      if  state_Goto = Stop then begin
        state_Follow := Follow_line;
        SetRCValue(20,1, format('STOP', []));
      end;
    end;

    Stop_Follow: begin
      if erro_dist > DIST_NEWPOSE then begin
        state_Follow := Follow_line;
      end;
    end;
  end;

  //Outputs
  case state_Follow of
    Follow_line: begin
      linear_vel := LIN_VEL_NOM;

      if abs(GAIN_DIST*dist2line - GAIN_FWD*erro_theta) > W_NOM  then
        angular_vel := sign(GAIN_DIST*dist2line - GAIN_FWD*erro_theta)*W_NOM
      else
        angular_vel := GAIN_DIST*dist2line - GAIN_FWD*erro_theta;
      (*
      if GAIN_DIST*dist2line - GAIN_FWD*erro_theta > 0 then
          angular_vel := Min(GAIN_DIST*dist2line - GAIN_FWD*erro_theta, W_NOM)
      else
          angular_vel := Max(GAIN_DIST*dist2line - GAIN_FWD*erro_theta, W_NOM
      *)
    end;

    De_Accel_Follow: begin
      linear_vel := LIN_VEL_DA;
      angular_vel := GAIN_DIST*dist2line - GAIN_FWD*erro_theta;
    end;

    Goto_Near_Point: begin
      gotoXYTheta(xn, yn, ang_line);
    end;

    Goto_XiYi: begin
      gotoXYTheta(xi, yi, ang_line);
    end;

    Goto_XaYa: begin
      gotoXYTheta(xa, ya, ang_line);
    end;

    Stop_Follow: begin
      linear_vel := 0;
      angular_vel := 0;
    end;
  end;

  // calculate v1 and v2 using linear_vel and angular_vel
  v1 := linear_vel - angular_vel*b/2;
  v2 := linear_vel + angular_vel*b/2;

  followline_result := (i, v1, v2);

end;

(* function doFollowArc(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var x_ini, y_ini, teta_ini: double;
    x_end, y_end, teta_end: double;
    xc, yc: double;
    error_teta, path_arc, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, speed, max_speed, d: double;
    wanted_teta, completed, path_length: double;
begin
  (* TODO *)
  (*
    atype := atFollowArc;
    x := px_ini;
    y := py_ini;
    teta := pteta_ini * Pi / 180;
    path_teta := pteta_end * Pi / 180;
    radius := pradius;
    rteta_start := prteta_start * Pi / 180;
    rteta_end := prteta_end * Pi / 180;
    speed = pspeed;
  *)

  x_ini := action.x;
  y_ini := action.y;

  xc := x_ini - cos(action.rteta_start) * action.radius;
  yc := y_ini - sin(action.rteta_start) * action.radius;

  x_end := xc + cos(action.rteta_end) * action.radius;
  y_end := yc + sin(action.rteta_end) * action.radius;

  teta_ini := action.teta;
  teta_end := action.path_teta;
  path_arc := DiffAngle(action.rteta_end, action.rteta_start);

  ang := ATan2(RobotState.y - yc, RobotState.x - xc);
  rpy := sign(path_arc) * (action.radius - Dist(RobotState.y - yc, RobotState.x - xc));
  completed := DiffAngle(ang, action.rteta_start) / path_arc;

  if completed < 0 then begin
     completed := 0;
  end;

  if completed > 1 then  begin
     completed := 1;
  end;

  wanted_teta := NormalizeAngle(teta_ini + DiffAngle(teta_end, teta_ini) * completed);

  rpteta := DiffAngle(RobotState.teta, NormalizeAngle(ang + (Pi/2) * sign(path_arc)));
  error_teta := DiffAngle(RobotState.teta, wanted_teta);

  speed := action.speed;

  // limit acceleration
  max_speed := Dist(RobotState.last_v, RobotState.last_vn) + 1 * 0.025;
  if speed > max_speed then begin
     speed := max_speed;
  end;

  vx := speed;
  FHal.EMission1.Text:=FloatToStr(speed);
  vy := -2 * rpy;     //-2
  w := -(StrToFloat(FHal.EWFollowArc.Text)) * error_teta;  //1.5

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);

  result := (completed >= 0.97);
end; *)



// ORDEM DE SINCRONISMO
//1- receber posiçao do robot através da camara (UDP packet)
//2- calcular velocidades v1 e v2
//3- enviar as velocidades para o arduino (main)
//4- recebe-se as velocidades do arduino/robot (main)
//5- voltar a 1




end.

