unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, IniPropStorage, SdpoSerial, LCLIntf, LCLType,
  ComCtrls, ActnList, channels, math, lNet, Utils;

const

  // GotoXYTheta states
  Rotate = 0;
  Go_Forward = 1;
  De_Accel = 2;
  Final_Rot = 3;
  DeAccel_Final_Rot = 4;
  Stop = 5;

  b = 0.09;

  W_NOM = 3500;
  LIN_VEL_NOM =1000;
  LIN_VEL_DA = 900;
  GAIN_FWD = 8;
  GAIN_FWD_I = 0; //1
  GAIN_DA = 4;
  GAIN_DA_D = 1;
  GAIN_DIST = 10;
  W_DA = 1000;
  TOL_FINTHETA = 0.052;            //3 graus
  THETA_DA = 0.122;        //7 graus
  TOL_FINDIST = 3.5;
  DIST_DA = 10;
  MAX_ETF = 0.07;     // 4 graus
  DIST_NEWPOSE = 6;
  THETA_NEWPOSE = 0.122;    // 7 graus

  MAX_DIST2LINE = 0.3;
  NEAR_LINE = 0.25;

type
   Matrix2D = array of array of Double;
   DoubleArray = array of Double;
   IntArray = array of integer;
  { TFMain }

  TFMain = class(TForm)
    BStop: TButton;
    BOpenSerial: TButton;
    BCloseSerial: TButton;
    BSendRaw: TButton;
    BConfigSet: TButton;
    BSerialSend: TButton;
    BSendR1: TButton;
    BGo: TButton;
    BSendR2: TButton;
    BSendR3: TButton;
    Label30: TLabel;
    Label46: TLabel;
    STOPGOTO: TButton;
    GOTOXY_y: TEdit;
    GOTOXY_theta: TEdit;
    GOTOXY_x: TEdit;
    GOTOXYTHETA: TButton;
    CBComBlinkMute: TCheckBox;
    CBRawDebug: TCheckBox;
    CBAutoOpen: TCheckBox;
    EditBatVoltage1: TEdit;
    EditBatVoltage2: TEdit;
    EditBatVoltageRaw1: TEdit;
    EditBatVoltageRaw2: TEdit;
    EditM0SetAcc: TEdit;
    EditM0Speed1: TEdit;
    EditM0Speed2: TEdit;
    EditM1SetAcc: TEdit;
    EditM0Speed: TEdit;
    EditBatVoltageRaw: TEdit;
    EditBatVoltageConv: TEdit;
    EditM0SetSpeed: TEdit;
    EditM1Speed1: TEdit;
    EditM1Speed2: TEdit;
    EditRobot: TEdit;
    EditM1Speed: TEdit;
    EditM1SetSpeed: TEdit;
    EditRobotGOTOXY: TEdit;
    EditSendChannel: TEdit;
    EditSendValue: TEdit;
    EditSerialName: TEdit;
    EditSendRaw: TEdit;
    EditBatVoltageTresh: TEdit;
    EditIPR2: TEdit;
    EditIPR3: TEdit;
    EditUDPPort: TEdit;
    EditBatVoltage: TEdit;
    EditIPR1: TEdit;
    EditUDPSendPort: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label19: TLabel;
    Label2: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label25: TLabel;
    Label26: TLabel;
    Label27: TLabel;
    Label28: TLabel;
    Label29: TLabel;
    Label3: TLabel;
    Label31: TLabel;
    Label32: TLabel;
    Label33: TLabel;
    Label34: TLabel;
    Label35: TLabel;
    Label36: TLabel;
    Label37: TLabel;
    Label38: TLabel;
    Label39: TLabel;
    Label4: TLabel;
    Label40: TLabel;
    Label41: TLabel;
    Label42: TLabel;
    Label43: TLabel;
    Label44: TLabel;
    Label45: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    UDPRobot: TLUDPComponent;
    ShapeComBlink: TShape;
    ShapeUDPState: TShape;
    TabData: TTabSheet;
    UDP: TLUDPComponent;
    MemoDebug: TMemo;
    PageControl: TPageControl;
    Serial: TSdpoSerial;
    ShapeSerialState: TShape;
    StatusBar: TStatusBar;
    TabDebug: TTabSheet;
    TabConfig: TTabSheet;
    procedure BCloseSerialClick(Sender: TObject);
    procedure BConfigSetClick(Sender: TObject);
    procedure BGoClick(Sender: TObject);
    procedure BOpenSerialClick(Sender: TObject);
    procedure BSendRawClick(Sender: TObject);
    procedure BSendUDPClick(Sender: TObject);
    procedure GOTOXYTHETAClick(Sender: TObject);
    procedure EditSendRawKeyDown(Sender: TObject; var Key: Word;
      Shift: TShiftState);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure STOPGOTOClick(Sender: TObject);
    procedure UDPReceive(aSocket: TLSocket);
    procedure UDPRobotReceive(aSocket: TLSocket);
  private
    procedure Debug(s: string);
    procedure processFrame(channel: char; value: integer; source: integer; robot: integer);
    procedure SendUDPChannel(c: char; val: integer; i: integer);
    procedure ShowSerialState;
    procedure ShowUDPState;

    { private declarations }
  public
    BatVoltageTresh, BatVoltageConv: double;
    SerialChannels, UDPChannels: TChannels;
    BatVoltage: double;
    BatVoltageRaw: integer;
    procedure SendMessage(c: char; val: integer);
  end;
  procedure gotoXYTheta_control(i:integer; xf, yf, tf: double);
var
  FMain: TFMain;
  pos_robot: Matrix2D;
  flagsign: boolean;
  state_Goto: integer;
  erro_theta_last: double;
  integrative: double;
  linear_vel, angular_vel: double;
  v1: IntArray;
  v2: IntArray;
  Flag : integer;
  sign_previous : double;
  sign_dir : integer;
  sign_dir_f: integer;

implementation

{$R *.lfm}

{ TFMain }


procedure TFMain.ShowUDPState;
begin
  if UDP.Connected then begin
    ShapeUDPState.Brush.Color := clGreen;
  end else begin
    ShapeUDPState.Brush.Color := clRed;
  end;
end;


procedure TFMain.ShowSerialState;
begin
  if Serial.Active then begin
    ShapeSerialState.Brush.Color := clGreen;
  end else begin
    ShapeSerialState.Brush.Color := clRed;
  end;
end;

procedure TFMain.SendMessage(c: char; val: integer);
begin
  Serial.WriteData(c + IntToHex(dword(Val), 8));
end;


procedure TFMain.SendUDPChannel(c: char; val: integer; i: integer);
var addr, ip: string;
begin
  if i=1 then begin
      ip:= EditIPR1.Text;
  end else if i=2 then begin
      ip:= EditIPR2.Text;
  end else if i=3 then begin
      ip:= EditIPR3.Text;
  end;
  addr := ip + ':' + EditUDPSendPort.Text;
  UDP.SendMessage(c + IntToHex(dword(Val), 8), addr);
end;

procedure TFMain.BOpenSerialClick(Sender: TObject);
begin
  Serial.Device := EditSerialName.Text;
  Serial.Open;
  ShowSerialState();
end;


procedure TFMain.BSendRawClick(Sender: TObject);
var addr: string;
begin
  if Serial.Active then Serial.WriteData(EditSendRaw.Text);
  if UDP.Active then begin
    addr := EditIPR1.Text + ':' + EditUDPSendPort.Text;
    UDP.SendMessage(EditSendRaw.Text, addr);
  end;

end;

procedure TFMain.BSendUDPClick(Sender: TObject);
var s: string;
  v: integer;
  i: integer;
begin
  s := EditSendChannel.Text;
  if s <> '' then begin
    v := StrToIntDef(EditSendValue.Text, 0);
    if (sender as TButton).Name = 'BSendR1' then begin
       i:=1;
    end else if (sender as TButton).Name = 'BSendR2' then begin
       i:=2;
    end else if (sender as TButton).Name = 'BSendR3' then begin
       i:=3;
    end;
    SendUDPChannel(s[1], v, i);
  end;
end;

procedure TFMain.GOTOXYTHETAClick(Sender: TObject);
var robot, x_f, y_f, theta_f: integer;
begin
  GOTOXYTHETA.Enabled := False;

end;

procedure TFMain.EditSendRawKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = VK_RETURN then begin
    BSendRaw.Click();
  end;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  Serial.Close;
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  DefaultFormatSettings.DecimalSeparator := '.';
  SerialChannels := TChannels.Create(@processFrame);
  UDPChannels := TChannels.Create(@processFrame);
  SetLength(pos_robot, 3, 4);
  SetLength(v1, 3);
  SetLength(v2, 3);
  GOTOXYTHETA.Cancel := False;
end;

procedure TFMain.FormShow(Sender: TObject);
var port: integer;
begin
  ShowSerialState();
  BConfigSet.Click();
  if CBAutoOpen.Checked then BOpenSerial.Click();

  port := StrToIntDef(EditUDPPort.Text, 9632);
  UDP.Listen(port);
  EditUDPPort.Text := IntToStr(port);
  ShowUDPState();
  UDPRobot.Listen(9600);
  EditIPR1.Text := '192.168.1.84';
  EditIPR2.Text := '192.168.1.83';
  EditIPR3.Text := '192.168.1.82';
end;

procedure TFMain.FormDestroy(Sender: TObject);
begin
  UDPChannels.Free;
  SerialChannels.Free;
end;



procedure TFMain.Debug(s: string);
begin
  //MemoDebug.VertScrollBar.Position := MemoDebug.VertScrollBar.Range;
  MemoDebug.Text := MemoDebug.Text + s;
  MemoDebug.SelStart:=Length(MemoDebug.Text);
  while MemoDebug.Lines.Count > 1000 do begin
    MemoDebug.Lines.Delete(0);
  end;
end;


procedure TFMain.SerialRxData(Sender: TObject);
var s: string;
  robotip: integer;
begin
  s := Serial.ReadData;
  if s = '' then exit;

  SerialChannels.ReceiveData(s, robotip);

  if CBRawDebug.Checked then begin
    Debug(s);
  end;
end;

procedure TFMain.STOPGOTOClick(Sender: TObject);
begin
      state_Goto := Rotate;
      GOTOXYTHETA.Enabled := True;
      erro_theta_last:=0;
      integrative := 0.0;
      flagsign:=false;
      sign_dir:=0;
      sign_dir_f:=0;

end;



procedure TFMain.UDPReceive(aSocket: TLSocket);
var msg, Robot_IP: string;
  i: integer;
begin
  UDP.GetMessage(msg);
  if msg = '' then exit;

  Robot_IP := aSocket.PeerAddress;

  if Robot_IP = EditIPR1.Text then begin
       i:=1;
    end else if Robot_IP = EditIPR2.Text then begin
       i:=2;
    end else if Robot_IP = EditIPR3.Text then begin
       i:=3;
    end;

  UDPChannels.ReceiveData(msg, i);

  if CBRawDebug.Checked then begin
    Debug(msg);
  end;

end;

procedure TFMain.UDPRobotReceive(aSocket: TLSocket);
var
  msg, res, info, Robot_IP: string;
  n, i, id, robot: integer;
  A, aux: TStringArray;
  pos_x, pos_y, pos_z, pos_theta, gotoxy_robot, x_f, y_f, theta_f: integer;

begin

  UDPRobot.GetMessage(msg);

   //Label14.Caption :='ola';
  if msg <> '' then begin
   // Label15.Caption :='mensagem com conteudo';
    n:=0;
     A := msg.Split(';'); //cada info de localizacao esta separada por ;
    // if length(A) = 3 then begin   //está a receber info de 3 robots
        for i:=0 to (length(A)-1) do
          begin

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
              pos_robot[n][0]:= StrToInt(aux[0]);
              aux := aux[1].Split('z');
              pos_robot[n][1]:= StrToInt(aux[0]);
              aux := aux[1].Split('t');
              pos_robot[n][2]:= StrToInt(aux[0]);
              pos_robot[n][3]:= StrToInt(aux[1]);

          end;
          Label14.Caption := 'id: 23- robot 1';
          Label15.Caption := 'x: ' + pos_robot[0][0].ToString;
          Label16.Caption := 'y: ' + pos_robot[0][1].ToString;
          Label17.Caption := 'z: ' + pos_robot[0][2].ToString;
          Label18.Caption := 'theta: ' + pos_robot[0][3].ToString;

          Label19.Caption := 'id: 33- robot 2';
          Label20.Caption := 'x: ' + pos_robot[1][0].ToString;
          Label21.Caption := 'y: ' + pos_robot[1][1].ToString;
          Label22.Caption := 'z: ' + pos_robot[1][2].ToString;
          Label23.Caption := 'theta: ' + pos_robot[1][3].ToString;

          Label24.Caption := 'id: 43- robot 3';
          Label25.Caption := 'x: ' + pos_robot[2][0].ToString;
          Label26.Caption := 'y: ' + pos_robot[2][1].ToString;
          Label27.Caption := 'z: ' + pos_robot[2][2].ToString;
          Label28.Caption := 'theta: ' + pos_robot[2][3].ToString;

          if GOTOXYTHETA.Enabled=false then begin
             robot := StrToInt(EditRobotGOTOXY.Text);
             x_f := StrToInt(GOTOXY_x.Text);
             y_f := StrToInt(GOTOXY_y.Text);
             theta_f := StrToInt(GOTOXY_theta.Text);
             gotoXYTheta_control(robot-1, x_f, y_f, theta_f);
          end;
      //  unit1.gotoXYTheta(2, 1, 1, 0);
      //  EditM0Speed.Text := v1[0].ToString;
      //  EditM1Speed.Text := v2[0].ToString;
     //   Label29.Caption := v1[2].ToString;
     //   Label30.Caption := v2[2].ToString;



    // end;

  end;
end;

procedure gotoXYTheta_control(i:integer; xf, yf, tf: double);
  var erro_dist, erro_theta, erro_theta_f, xr_e, yr_e, tr_e, derivative: double;
    M0Speed, M1Speed, flag_rot, auxint: integer;
    M0Acc, M1Acc: integer;
    msg, addr, ip: string;
  begin

    //obter coordenadas do robot i
    xr_e:= pos_robot[i][0];
    yr_e:= pos_robot[i][1];
    tr_e := degtorad(pos_robot[i][3]);
    tf := degtorad(tf);


    //Calc errors
    erro_theta := NormalizeAngle(DiffAngle(tr_e, ATan2(yf - yr_e, xf - xr_e)));  //normalizar os dois angulos para 360

    FMain.Label32.Caption := 'erro theta: '+erro_theta.ToString;
    erro_dist := sqrt(power(xr_e - xf,2) + power(yr_e -yf,2));
    FMain.Label34.Caption := 'erro dist: '+erro_dist.ToString;
    erro_theta_f := NormalizeAngle(tf - tr_e);
    FMain.Label33.Caption := 'erro theta f: '+erro_theta_f.ToString;
   // sign_dir:= 1;
  //  sign_dir_f := 1;

     integrative := integrative + erro_theta;

     FMain.Label46.Caption := 'integrative: '+ integrative.ToString;

    (*if (abs(pos_robot[i][3]) > 165) and (abs(pos_robot[i][3]) < 181)  then begin
       flagsign:=true;
    end else begin
       flagsign:=false;
    end;

    //Find fastest Rotation
    if flagsign then begin
      sign_dir := sign_previous;
    end else if erro_theta > 0 then begin
      sign_dir := -1;
    end else  begin
      sign_dir := 1;
    end;

    sign_previous := sign_dir;   *)

   (* if (abs(erro_theta)>degtorad(170)) then begin
      sign_dir := 1;
    end;

     if (abs(erro_theta_f)>degtorad(170)) then begin
      sign_dir_f := 1;
    end;  *)


    //Find fastest Rotation

    if flagsign = false then begin
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
    end;

    derivative := erro_theta - erro_theta_last;
    erro_theta_last:= erro_theta;

    //Transitions
    case state_Goto of
      Rotate: begin
        if (abs(erro_theta) < MAX_ETF) then begin
          state_Goto := Go_Forward;
        end else if erro_dist < TOL_FINDIST then begin
          state_Goto := Final_Rot;
        end;
      end;

      Go_Forward: begin
        if erro_dist < DIST_DA then begin
          state_Goto := De_Accel;
       // end else if abs(erro_theta) > MAX_ETF then begin
       //   state_Goto := Rotate;
        end;
      end;

      De_Accel: begin
        if erro_dist < TOL_FINDIST then state_Goto := Final_Rot;
      end;

      Final_Rot: begin
        if abs(erro_theta_f) < THETA_DA then begin
          state_Goto := DeAccel_Final_Rot;
        end else begin
          if erro_dist > DIST_NEWPOSE then state_Goto := Rotate;
        end;
      end;

      DeAccel_Final_Rot: begin
        if abs(erro_theta_f) < TOL_FINTHETA then begin
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
        flagsign := true; //locks the possibility to change rotation direction
        linear_vel := 0;
        angular_vel := sign_dir*W_NOM;
      end;

      Go_Forward : begin
        flagsign := false;
        linear_vel := LIN_VEL_NOM;
        //angular_vel := -GAIN_FWD*erro_theta-GAIN_FWD_D*derivative;
        //- GAIN_FWD_I*integrative;
        angular_vel := -GAIN_FWD*erro_theta - GAIN_FWD_I*integrative;
      end;

      De_Accel : begin
        linear_vel := LIN_VEL_DA;
      //  angular_vel := -GAIN_DA*erro_theta - GAIN_DA_D*derivative;
        angular_vel := -GAIN_DA*erro_theta;
      end;

      Final_Rot : begin
        flagsign := true;
        linear_vel := 0;
        angular_vel := sign_dir_f*W_NOM;
      end;

      DeAccel_Final_Rot : begin
        flagsign := false;
        linear_vel := 0;
        angular_vel := sign_dir_f*W_DA;
      end;

      Stop : begin
        flagsign := false;
        linear_vel := 0;
        angular_vel := 0;
      end;
    end;

    FMain.Label31.Caption := 'State: ' + IntToStr(state_Goto);
    // calculate v1 and v2 using linear_vel and angular_vel
    v1[i] := -(Trunc(linear_vel + angular_vel*b/2));
    v2[i] := Trunc(linear_vel - angular_vel*b/2);
    FMain.Label29.Caption := 'v1: '+v1[i].ToString;
    FMain.Label30.Caption := 'v2: '+v2[i].ToString;

    (*M0Speed := -(Trunc(linear_vel + angular_vel*b/2));
    M1Speed := Trunc(linear_vel - angular_vel*b/2);     *)

    M0Speed := -(Trunc(linear_vel + angular_vel*b/2));
    M1Speed := Trunc(linear_vel - angular_vel*b/2);

    M0Acc := 100;
    M1Acc := 100;

    if (i+1) = 1 then begin
       ip:= FMain.EditIPR1.Text;
    end else if (i+1)=2 then begin
        ip:= FMain.EditIPR2.Text;
    end else if (i+1)=3 then begin
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



end;

procedure TFMain.BCloseSerialClick(Sender: TObject);
begin
  Serial.Close;
  ShowSerialState();
end;

procedure TFMain.BConfigSetClick(Sender: TObject);
begin
  BatVoltageConv := StrToFloat(EditBatVoltageConv.Text);
  BatVoltageTresh := StrToFloat(EditBatVoltageTresh.Text);
end;

procedure TFMain.BGoClick(Sender: TObject);
var M0Speed, M1Speed: integer;
    M0Acc, M1Acc: integer;
    msg, addr, ip: string;
begin

  if (sender as TButton).Cancel then begin
   // Flag := 1;

    M0Speed := 0;
    M1Speed := 0;

    M0Acc := 255;
    M1Acc := 255;

  end else begin
    M0Speed := StrToIntDef(EditM0SetSpeed.Text, 0);
    M1Speed := StrToIntDef(EditM1SetSpeed.Text, 0);

    M0Acc := StrToIntDef(EditM0SetAcc.Text, 100);
    M1Acc := StrToIntDef(EditM1SetAcc.Text, 100);

  end;

  if StrToInt(EditRobot.Text) = 1 then begin
       ip:= EditIPR1.Text;
  end else if StrToInt(EditRobot.Text)=2 then begin
      ip:= EditIPR2.Text;
  end else if StrToInt(EditRobot.Text)=3 then begin
      ip:= EditIPR3.Text;
  end;

  addr :=  ip + ':' + EditUDPSendPort.Text;
  msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
  msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
  UDP.SendMessage(msg, addr);
end;



procedure TFMain.processFrame(channel: char; value: integer; source: integer; robot: integer);
var i: integer;
    s: string;
begin
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'i' then begin
  //end else if channel = 'r' then begin
    // if the arduino was reset ...
  end else if channel = 'V' then begin
    BatVoltageRaw := value;
    BatVoltage := value * BatVoltageConv;

    if robot =1 then begin
       EditBatVoltageRaw.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage.Text := format('%.2f', [BatVoltage]);
    end else if robot =2 then begin
       EditBatVoltageRaw1.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage1.Text := format('%.2f', [BatVoltage]);
    end else if robot =3 then begin
       EditBatVoltageRaw2.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage2.Text := format('%.2f', [BatVoltage]);
    end;


    if not CBComBlinkMute.Checked then begin
      if ShapeComBlink.Brush.Color <> clGreen then begin
        ShapeComBlink.Brush.Color := clGreen;
      end else begin
        ShapeComBlink.Brush.Color := clLime;
      end;
    end;
  end else if channel = 'M' then begin //motor 1
    if robot =1 then begin
      EditM0Speed.Text := format('%d', [value]);
    end else if robot =2 then begin
      EditM0Speed1.Text := format('%d', [value]);
    end else if robot = 3 then begin
      EditM0Speed2.Text := format('%d', [value]);
    end;
  end else if channel = 'N' then begin  //motor 2
      if robot =1 then begin
      EditM1Speed.Text := format('%d', [value]);
    end else if robot =2 then begin
      EditM1Speed1.Text := format('%d', [value]);
    end else if robot = 3 then begin
      EditM1Speed2.Text := format('%d', [value]);
    end;
  end else if channel in ['s', 't'] then begin
    i := 1 + ord(channel) - ord('r');

  end;
end;



end.

