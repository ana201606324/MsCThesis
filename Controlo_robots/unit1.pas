unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls,
  lNetComponents, lNet, math, utils;

const

  // GotoXYTheta states
  Rotate = 0;
  Go_Forward = 1;
  De_Accel = 2;
  Final_Rot = 3;
  DeAccel_Final_Rot = 4;
  Stop = 5;

  b = 0.09;

  W_NOM = 2000;
  LIN_VEL_NOM = 1000;
  LIN_VEL_DA = 900;
  GAIN_FWD = 10;
  GAIN_DA = 100;
  GAIN_DIST = 500;
  W_DA = 2;
  TOL_FINTHETA = 0.052;            //3 graus
  THETA_DA = 0.122;        //7 graus
  TOL_FINDIST = 3;
  DIST_DA = 5;
  MAX_ETF = 0.07;     // 4 graus
  DIST_NEWPOSE = 7;
  THETA_NEWPOSE = 0.122;    // 7 graus

  MAX_DIST2LINE = 0.3;
  NEAR_LINE = 0.25;

type
   Matrix2D = array of array of integer;
   DoubleArray = array of Double;
  { TForm1 }

  TForm1 = class(TForm)
    Button1: TButton;
    Edit1: TEdit;
    Edit2: TEdit;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    UDPComp: TLUDPComponent;
  //  procedure Edit1Change(Sender: TObject);
    procedure FormCreate(Sender: TObject);
 //   procedure FormDestroy(Sender: TObject);
   // procedure Label1Click(Sender: TObject);
   // procedure LUDPComponent1Error(const msg: string; aSocket: TLSocket);
  //  procedure UDPCompReceive(aSocket: TLSocket);

  private

  public
    pos_robot: Matrix2D;


  end;
   procedure gotoXYTheta(i:integer; xf, yf, tf: double);
var
  Form1: TForm1;
  state_Goto: integer;
  linear_vel, angular_vel: double;
//  v1: DoubleArray;
//  v2: DoubleArray;

implementation
     uses main;
{$R *.lfm}

procedure gotoXYTheta(i:integer; xf, yf, tf: double);
  var sign_dir, sign_dir_f, erro_dist, erro_theta, erro_theta_f, xr_e, yr_e, tr_e: double;
    M0Speed, M1Speed: integer;
    M0Acc, M1Acc: integer;
    msg, addr, ip: string;
  begin
   (*
    //obter coordenadas do robot i
    xr_e:= pos_robot[i][0];
    yr_e:= pos_robot[i][1];
    tr_e := degtorad(pos_robot[i][3]);

    //Calc errors
    erro_theta := NormalizeAngle(tr_e - ATan2(yf - yr_e, xf - xr_e));
    //FMain.Label32.Caption := 'erro theta: '+erro_theta.ToString;
    erro_dist := sqrt(power(xr_e - xf,2) + power(yr_e -yf,2));
    erro_theta_f := NormalizeAngle(degtorad(tf) - tr_e);
   // FMain.Label33.Caption := 'erro theta f '+erro_theta_f.ToString;

    //Find fastest Rotation
    if erro_theta > 0 then begin    //>0
      sign_dir := -1;
    end else begin
      sign_dir := 1;
    end;

    if erro_theta_f > 0 then begin  //>0
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
        if erro_dist < DIST_DA then begin
          state_Goto := De_Accel;
      (*  end else if abs(erro_theta) > MAX_ETF then begin
          state_Goto := Rotate;  *)
        end;
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

    Label31.Caption := 'State: ' + IntToStr(state_Goto);
    // calculate v1 and v2 using linear_vel and angular_vel
    v1[i] := -Trunc(linear_vel - angular_vel*b/2);
    v2[i] := Trunc(linear_vel + angular_vel*b/2);
    Label29.Caption := 'v1: '+v1[i].ToString;
    Label30.Caption := 'v2: '+v2[i].ToString;

    M0Speed := -Trunc(linear_vel - angular_vel*b/2);
    M1Speed := Trunc(linear_vel + angular_vel*b/2);

    M0Acc := 100;
    M1Acc := 100;

    if i = 1 then begin
       ip:= FMain.EditIPR1.Text;
    end else if i=2 then begin
        ip:= FMain.EditIPR2.Text;
    end else if i=3 then begin
        ip:= FMain.EditIPR3.Text;
    end;

    addr :=  ip + ':' + FMain.EditUDPSendPort.Text;
    msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
    msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
    FMain.UDP.SendMessage(msg, addr);

  (*  if FMain.Flag <> 1 then begin
       FMain.EditM0SetSpeed.Text := FMain.v1[i].ToString;
       FMain.EditM1SetSpeed.Text := FMain.v2[i].ToString;
       FMain.BGo.Click;
    end; *)

 (*   M0Speed := FMain.v1[i];
    M1Speed := FMain.v2[i];

    M0Acc := StrToIntDef(FMain.EditM0SetAcc.Text, 100);
    M1Acc := StrToIntDef(FMain.EditM1SetAcc.Text, 100);

    if FMain.Flag <> 1 then begin
      FMain.EditUDPIP.Text := '192.168.1.122';
      addr := FMain.EditUDPIP.Text + ':' + FMain.EditUDPSendPort.Text;
      msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
      msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
      FMain.UDP.SendMessage(msg, addr);
    end;    *)

    *)

end;

{ TForm1 }


procedure TForm1.FormCreate(Sender: TObject);
begin
 // SetLength(pos_robot, 3, 4);
  //if UDPComp.Listen(9800) then begin
  //   Edit1.Text:= 'Listening';
 // end;
end;




(*


procedure TForm1.UDPCompReceive(aSocket: TLSocket);
var
  msg, res, info: string;
  n, i, id: integer;
  A, aux: TStringArray;
  pos_x, pos_y, pos_z, pos_theta: double;
begin
  UDPComp.GetMessage(msg);

  if msg <> '' then begin
     A := msg.Split(';'); //cada info de localizacao esta separada por ;
     if length(A) = 3 then begin   //está a receber info de 3 robots
        for i:=0 to 2 do
          begin
            //pos_robot[i][0] := 1;
              aux := A[i].Split('x');
              res := aux[0];
              Delete(res, 1,1);
              id := StrToInt(res);

              if id = 23 then begin
                 n:=0;
              end
              else if id = 33 then begin
                 n:=1;
              end
              else if id = 43 then begin
                 n:=2;
              end;

              aux := aux[1].Split('y');
              pos_robot[n][0] := aux[0].ToDouble;
              aux := aux[1].Split('z');
              pos_robot[n][1] := aux[0].ToDouble;
              aux := aux[1].Split('t');
              pos_robot[n][2] := aux[0].ToDouble;
              pos_robot[n][3] := aux[1].ToDouble;



          end;
        Label1.Caption := 'id 23';
        Label2.Caption := 'x: ' + pos_robot[0][0].ToString;
        Label3.Caption := 'y: ' + pos_robot[0][1].ToString;
        Label4.Caption := 'z: ' + pos_robot[0][2].ToString;
        Label5.Caption := 'theta: ' + pos_robot[0][3].ToString;
        Label6.Caption := 'id 33';
        Label7.Caption := 'x: ' + pos_robot[1][0].ToString;
        Label8.Caption := 'y: ' + pos_robot[1][1].ToString;
        Label9.Caption := 'z: ' + pos_robot[1][2].ToString;
        Label10.Caption := 'theta: ' + pos_robot[1][3].ToString;
        Label11.Caption := 'id 43';
        Label12.Caption := 'x: ' + pos_robot[2][0].ToString;
        Label13.Caption := 'y: ' + pos_robot[2][1].ToString;
        Label14.Caption := 'z: ' + pos_robot[2][2].ToString;
        Label15.Caption := 'theta: ' + pos_robot[2][3].ToString;

      //  Application.CreateForm(TFControlo, FControlo);
        gotoXYTheta(0, 1.0, 1.0, 90.0);
        FMain.EditM0SetSpeed.Text := v1[0].ToString;
        FMain.EditM1SetSpeed.Text := v2[0].ToString;


     end;

  end;
end; *)

end.

