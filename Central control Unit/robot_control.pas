unit robot_control;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, IniPropStorage, SdpoSerial, LCLIntf, LCLType,
  ComCtrls, channels, math, lNet, Utils, Robot_Configuration;




  const

 // GotoXYTheta states
  Rotate = 0;
  Go_Forward = 1;
  De_Accel = 2;
  Final_Rot = 3;
  DeAccel_Final_Rot = 4;
  Stop = 5;

  b = 9;  //cm

  W_NOM = 100;
  LIN_VEL_NOM =800;
  LIN_VEL_DA = 600;
  GAIN_FWD = 70;
  GAIN_FWD_I = 0; //1
  GAIN_DA = 50;
  GAIN_DA_FINAL= GAIN_DA*0.75 ;
  GAIN_DIST = 10;
  W_DA = 1500;
  TOL_FINTHETA = 0.052;            //3 graus
  THETA_DA = 0.122;        //7 graus
  TOL_FINDIST = 3;       //cm
  DIST_DA = 20;            //cm
  MAX_ETF = 0.07;     // 4 graus
  DIST_NEWPOSE = 15;       //cm
  THETA_NEWPOSE = 0.122;    // 7 graus



  var
  goto_result:array[1..3] of double;
  followline_result:array[1..3] of double;
  v1, v2: double;
  linear_vel, angular_vel: double;
  stateGT:array[1..3] of integer;
  dir:array[1..3] of integer;
  flagsign:array[1..3] of boolean;
  xr_e, yr_e, tr_e: double;
  position_robot:array[1..3, 1..3] of double;



  X:array[1..3, 1..15] of double; //matriz com todas as coordenadas x de todos os steps(consoante a coluna) de cada robot(consoante a linha)
  Y:array[1..3, 1..15] of double;
  D:array[1..3, 1..15] of double;


  ind:integer;
  robot_i: integer;
  C_a:array[1..3] of integer;
  Count_s:array[1..3] of integer;
  target:array[1..3] of integer;
  steps_a:array[1..3] of integer;
  flag_new:array[1..3] of boolean;
  estado:array[1..3] of integer;

  procedure gotoXYTheta_control(i:integer; xr, yr, angler, xf, yf, tf: double);
  procedure robot_control_main(N,P,S:integer; matrix:Matrix2D; T:integer);

  implementation
uses
  controlo;

procedure gotoXYTheta_control(i:integer; xr, yr, angler, xf, yf, tf: double);
  var erro_dist, erro_theta, erro_theta_f, xr_e, yr_e, tr_e, q: double;
    M0Speed, M1Speed, flag_rot, auxint, rot, a, b,c, k, stat: integer;
    M0Acc, M1Acc, stopv: integer;
    msg, addr, ip: string;
  begin


    i:=i-1;
    if flaginit=true then begin
      for k:=0 to 2 do begin
        robot_control.flagsign[k]:=false;
        robot_control.stateGT[k] := 0;
        estado[k]:=0;
        robot_control.dir[k]:=0;
      end;
      flaginit:=false;
   end;

    xf:=xf*100;
    yf:=yf*100;
    //obter coordenadas do robot i
    xr_e:= xr*100;
    yr_e:= yr*100;
    tr_e := angler;


    auxint:= round(tf);

    case auxint of
         0:begin
            tf := pi;
         end;
         1:begin
            tf := pi*0.75;
         end;
         2:begin
            tf := pi/2;
         end;
         3:begin
            tf := pi/4;
         end;
         4:begin
            tf := 0;
         end;
         5:begin
            tf := -pi/4;
         end;
         6:begin
            tf := -pi/2;
         end;
         7:begin
            tf := -3*pi/4;
         end;
    end;
    //tf := degtorad(tf); //DIRECTION??


    //Calc errors
    erro_theta := NormalizeAngle(DiffAngle(tr_e, ATan2(yf - yr_e, xf - xr_e)));  //normalizar os dois angulos para 360

    FRobot_Configuration.Label39.Caption := 'erro theta: '+RadToDeg(erro_theta).ToString;
    erro_dist := sqrt(power(xr_e - xf,2) + power(yr_e -yf,2));
    //FMain.Label34.Caption := 'erro dist: '+erro_dist.ToString;
    erro_theta_f := NormalizeAngle(DiffAngle(tf, tr_e));
    //FMain.Label33.Caption := 'erro theta f: '+erro_theta_f.ToString;


     integrative := integrative + erro_theta;

     //FMain.Label46.Caption := 'integrative: '+ integrative.ToString;

     if i=0 then begin
       k:=0;
     end;

    //Find fastest Rotation

    if robot_control.flagsign[i] = false then begin
      if erro_theta > 0 then begin
        robot_control.dir[i]:=-1;
      end else begin
        robot_control.dir[i]:=1;
      end;
    end;



    //Transitions
   if estado[i] = robot_control.Rotate then begin
        if (abs(erro_theta) < robot_control.MAX_ETF) then begin
          estado[i] := robot_control.Go_Forward;
        end else if erro_dist < robot_control.TOL_FINDIST then begin
          estado[i] := robot_control.Final_Rot;
        end;

   end else if estado[i] = robot_control.Go_Forward then begin
        if erro_dist < robot_control.DIST_DA then begin
         estado[i] := robot_control.De_Accel;
       // end else if abs(erro_theta) > MAX_ETF then begin
       //   state_Goto := Rotate;
        end;

   end else if estado[i] = robot_control.De_Accel then begin
        if erro_dist < robot_control.TOL_FINDIST then estado[i] := robot_control.Final_Rot;

   end else if estado[i] =robot_control.Final_Rot then begin
        if abs(erro_theta_f) < robot_control.THETA_DA then begin
          estado[i]:= robot_control.DeAccel_Final_Rot;
        end else begin
          if erro_dist > robot_control.DIST_NEWPOSE then estado[i] := robot_control.Rotate;
        end;

   end else if estado[i] = robot_control.DeAccel_Final_Rot then begin
        if abs(erro_theta_f) < robot_control.TOL_FINTHETA then begin
          estado[i] := robot_control.Stop;
        end else begin
          if erro_dist > robot_control.DIST_NEWPOSE then begin
              estado[i] := robot_control.Rotate;
              robot_control.flagsign[i] := false;
          end;
        end;
   end else if estado[i] =robot_control.Stop then begin
        if (abs(erro_theta_f) > robot_control.THETA_NEWPOSE) or (erro_dist > robot_control.DIST_NEWPOSE) then begin
          estado[i] := robot_control.Rotate;
        end;
   end;


    //Outputs
    if estado[i] = robot_control.Rotate then begin
            robot_control.flagsign[i] := true; //locks the possibility to change rotation direction
            linear_vel := 0;
            angular_vel := robot_control.dir[i]*W_NOM;
    end else if estado[i] = robot_control.Go_Forward then begin
            robot_control.flagsign[i] := false;
            linear_vel := LIN_VEL_NOM;
            //angular_vel := -GAIN_FWD*erro_theta-GAIN_FWD_D*derivative;
            //- GAIN_FWD_I*integrative;
            angular_vel := -GAIN_FWD*erro_theta - GAIN_FWD_I*integrative;
            //FMain.Label47.Caption := 'angular vel '+angular_vel.ToString;

    end else if estado[i] = robot_control.De_Accel then begin

            robot_control.linear_vel := LIN_VEL_DA*erro_dist/(DIST_DA);
          //  angular_vel := -GAIN_DA*erro_theta - GAIN_DA_D*derivative;
            robot_control.angular_vel := -GAIN_DA*erro_theta;
    end else if estado[i] = robot_control.Final_Rot then begin
            robot_control.flagsign[i] := true;
            robot_control.linear_vel := 0;
            robot_control.angular_vel := GAIN_DA*erro_theta_f;//sign_dir_f*W_NOM;
    end else if estado[i] = robot_control.DeAccel_Final_Rot then begin
            robot_control.linear_vel := 0;
            robot_control.angular_vel := GAIN_DA_FINAL*erro_theta_f;//sign_dir_f*W_DA;
    end else if estado[i] = robot_control.Stop then begin
            robot_control.flagsign[i] := false;
            robot_control.linear_vel := 0;
            robot_control.angular_vel := 0;
    end;
   //  angular_vel:=1000;
   //  linear_vel:=0;


      a:=estado[0];
      b:=estado[1];
      c:=estado[2];
     FRobot_Configuration.Label40.Caption := 'State: ' + IntToStr(a);
     FRobot_Configuration.Label41.Caption := 'State: ' + IntToStr(b);
     FRobot_Configuration.Label42.Caption := 'State: ' + IntToStr(c);
    // calculate v1 and v2 using linear_vel and angular_vel
   // v1[i] := -(Trunc(linear_vel + angular_vel*b/2));
   // v2[i] := Trunc(linear_vel - angular_vel*b/2);
    //FMain.Label29.Caption := 'v1: '+v1[i].ToString;
    //FMain.Label30.Caption := 'v2: '+v2[i].ToString;

    (*M0Speed := -(Trunc(linear_vel + angular_vel*b/2));
    M1Speed := Trunc(linear_vel - angular_vel*b/2);     *)

    M0Speed := -(Trunc(robot_control.linear_vel + robot_control.angular_vel*robot_control.b/2));
    M1Speed := Trunc(robot_control.linear_vel - robot_control.angular_vel*robot_control.b/2);

    M0Acc := 100;
    M1Acc := 100;

    if (i+1) = 1 then begin
       ip:= FRobot_Configuration.EditIPR1.Text;
       FRobot_Configuration.Label30.Caption:= M0Speed.ToString;
       FRobot_Configuration.Label31.Caption:= M1Speed.ToString;
    end else if (i+1)=2 then begin
        ip:= FRobot_Configuration.EditIPR2.Text;
        FRobot_Configuration.Label32.Caption:= M0Speed.ToString;
        FRobot_Configuration.Label33.Caption:= M1Speed.ToString;
    end else if (i+1)=3 then begin
        ip:= FRobot_Configuration.EditIPR3.Text;
        FRobot_Configuration.Label34.Caption:= M0Speed.ToString;
        FRobot_Configuration.Label35.Caption:= M1Speed.ToString;
    end;

    addr :=  ip + ':' + FRobot_Configuration.EditUDPSendPort.Text;
    msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
    msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
    //FRobot_Configuration.UDP.SendMessage(msg, addr);








end;





// ORDEM DE SINCRONISMO
//1- receber posiçao do robot através da camara (UDP packet)
//2- calcular velocidades v1 e v2
//3- enviar as velocidades para o arduino (main)
//4- recebe-se as velocidades do arduino/robot (main)
//5- voltar a 1


procedure robot_control_main(N,P,S:integer; matrix:Matrix2D; T:integer);
 var aux: double;
   dire, xr, yr, angler: double;
   verifica: boolean;
   robot_i, ind: integer;
begin

  xr:=pos_robot[N-1][0];
  yr:=pos_robot[N-1][1];
  angler:=pos_robot[N-1][3];
  robot_i:=N;
  flag_new[robot_i-1]:=false;

  robot_control.C_a[robot_i-1]:=1;
  robot_control.Count_s[robot_i-1]:=1;
  robot_control.target[robot_i-1]:=T;
  robot_control.steps_a[robot_i-1]:= S;
  //X[i][ind] --> pos X do no ind para o robot i

  for ind:=0 to 14 do begin   //pois so mandamos o maximo de 15 steps

       robot_control.X[robot_i-1][ind]:=matrix[ind][0];
       robot_control.Y[robot_i-1][ind]:=matrix[ind][1];
       robot_control.D[robot_i-1][ind]:=matrix[ind][2];
  end;

  ind:=0;
 (* while flag_new[robot_i-1]=false do
  begin *)
  FRobot_Configuration.Label38.Caption:= 'id: '+robot_i.ToString+' Xf: '+ robot_control.X[robot_i-1][ind].ToString+' Yf: '+ robot_control.Y[robot_i-1][ind].ToString+' Df: '+robot_control.D[robot_i-1][ind].ToString;

  //if(state_Goto[robot_i-1]=4) then verifica:=true;

  gotoXYTheta_control(robot_i, xr, yr, angler, robot_control.X[robot_i-1][ind], robot_control.Y[robot_i-1][ind], robot_control.D[robot_i-1][ind]);
 // gotoXYTheta_control(2, xr, yr, angler, xr, yr, 0);
 // gotoXYTheta_control(3, pos_robot[2][0], pos_robot[2][1], pos_robot[2][3], 1, 2, 0);
 (* if(state_Goto[robot_i-1]=4) and (verifica=true) then begin
    verifica:=false;
    ind:=ind+1;
     gotoXYTheta_control(2, 0, 0.5, 6);
    //gotoXYTheta_control(robot_i, X[robot_i-1][ind], Y[robot_i-1][ind], dire);
  end;*)

 (*      if (state_Goto[robot_i-1]=4) and (ind<steps_a[robot_i]-1) then begin
         ind:=ind+1;
       end else if (state_Goto[robot_i-1]=4) and (ind=steps_a[robot_i-1]-1) then break;
       if flag_new[robot_i-1]=true then break;
  end;   *)

  robot_i:=0;
  ind:=0;
  //flagcoms:=1;
  //flag1:=1

end;


end.

