program project1;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, GLScene_RunTime, sdposeriallaz, lnetvisual, Unit1, Unit2, main,
  controlo, Utils, Unit3, Robot_Configuration, udp_pc, unit4
  { you can add units after this };

{$R *.res}

begin
  RequireDerivedFormResource:=True;
  Application.Scaled:=True;
  Application.Initialize;
  //Application.CreateForm(TFRobot_Configuration, FRobot_Configuration);
  Application.CreateForm(TForm1, Form1);
  Application.CreateForm(TForm2, Form2);
  Application.CreateForm(TForm3, Form3);
  //Application.CreateForm(TForm3, Form3);
  //Application.CreateForm(TFMain, FMain);
  Application.Run;
end.

