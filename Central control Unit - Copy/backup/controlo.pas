unit controlo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, lNet, TEAstar, laz2_DOM, laz2_XMLRead, math, Utils3DS,Utils;

const
  THRESHOLD_DIST = 0.033;//0.025;
  THRESHOLD_ANGLE = 0.34;   //0.33
  //THRESHOLD_ANGLE_BACK = 0.3;
  CELLSCALE = 1;
  VNOM = 5.5 ;
  WNOM = 40;  //20
  GANHO_DIST = 240;    //320
  GANHO_THETA = 60;    //40
  GANHO_DIST_CIRCLE = 340;  //340
  GANHO_THETA_CIRCLE = 30;  //30
  GANHO_DIST_CIRCLE_BACK = 260;
  GANHO_THETA_CIRCLE_BACK = 80;
  RAIO_CURVATURA = CELLSCALE;

  //GANHOS DIMENSIONADOS PARA VNOM=1.3
  //240:60:400:40:480:30

type

  { TFControlo }

  Coordinates = record
    x, y: double;
  end;

   Coms_flaws_per_robot =record
     isactive:integer;
     isdetecting:integer;
     active_consecutive_hits:integer;
     curr_in_node:integer;
     curr_out_node:integer;
     flaw_ind:integer;
   end;
   coms_flaw_location =record
      n_flaws:integer;
      in_node:array of array of integer;
      unvin_node:array of array of  array of integer;
      detected_nodes:array of array of integer;
   end;
  TFControlo = class(TForm)
    Edit10: TEdit;
    Edit2: TEdit;
    Edit3: TEdit;
    Edit4: TEdit;
    Edit5: TEdit;
    Edit6: TEdit;
    Edit7: TEdit;
    Edit8: TEdit;
    Edit9: TEdit;
    LabeledEdit1: TLabeledEdit;
    SendButton: TButton;
    Edit1: TEdit;
    TimerSend: TTimer;
    udpCom: TLUDPComponent;
    //procedure Coms_timeout_timerTimer(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SendButtonClick(Sender: TObject);
    procedure TimerSendTimer(Sender: TObject);
    procedure udpComReceive(aSocket: TLSocket);
  private
    { private declarations }
  public
    { public declarations }
  //robots_flaws:array [0..NUMBER_ROBOTS-1] of  Coms_flaws_per_robot;
  end;


   r_node=TEAstar.r_node;
   m_node=array[0..NUMBER_ROBOTS-1] of integer;
   i_array=array of integer;
   cf_robots = array[0..NUMBER_ROBOTS-1] of Coms_flaws_per_robot;
   ii_array=array of array of integer;


    procedure InitialPointsForAllRobots(var agvs:r_node);
    procedure ChangeRobotPriorities(var Map:TAStarMap;var agvs:r_node);
    procedure InverseValidationOfPriorities(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
    procedure TEArun(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
    //function DistToReference(robot:integer;xCam:DOUBLE;yCam:double):double;
    //function AngleToReference(robot:integer;xCam:double;yCam:double;thetaCam:double):double;
    //function Signal(value:integer):integer;
    //procedure UpdateThetaDest(robot:integer;thetaCam:double);
    //procedure UpdateThetaDestPi(robot:integer;thetaCam:double);
    //procedure UpdateThetaDestToMoveBack(robot:integer;thetaCam:double);
    //procedure UpdateThetaDestAfterPiRotation(robot:integer;thetaCam:double);
    procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer ;data:string);
    procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer);
    procedure UpdateSubmissions(var agvs:r_node;i:integer);











var
  FControlo: TFControlo;
  MessageInitialPositions: string;
  MessageVelocities: string;
  MessageVelocities1: string;
  Map: TAStarMap;
  agvs: r_node;
  CaminhosAgvs: Caminhos;
  CaminhosAgvs_af: Caminhos;
  CaminhosAgvs_a: Caminhos;
  CaminhosAgvs_s: Caminhos;
  Ca: array[0..NUMBER_ROBOTS-1] of integer;
  flagMessageInitialPositions: boolean;
  flagVelocities: boolean;
  xDest: array[0..NUMBER_ROBOTS-1] of double;
  yDest: array[0..NUMBER_ROBOTS-1] of double;
  thetaDest: array[0..NUMBER_ROBOTS-1] of double;
  directionDest: array[0..NUMBER_ROBOTS-1] of integer;
  contador: double;
  linearVelocities: array[0..NUMBER_ROBOTS-1] of double;
  angularVelocities: array[0..NUMBER_ROBOTS-1] of double;
  followLine: array[0..NUMBER_ROBOTS-1] of boolean;
  followCircle: array[0..NUMBER_ROBOTS-1] of boolean;
  rotate: array[0..NUMBER_ROBOTS-1] of boolean;
  robotNoPlan: integer;
  Doc: TXMLDocument;
  directionToFollow: array[0..NUMBER_ROBOTS-1] of integer;
  target: array[0..NUMBER_ROBOTS-1] of boolean;
  rotationCenter: array[0..NUMBER_ROBOTS-1] of Coordinates;
  s: array[0..NUMBER_ROBOTS-1] of Coordinates;
  dist,angle:double;
  wf: array[0..NUMBER_ROBOTS-1] of double;
  flagChange: boolean;
  ListPriorities: array[0..NUMBER_ROBOTS-1] of integer;
  //mantém-se a ordem inicial dos robôs e guarda-se a prioridade atual
  totalTrocas: integer;
  totalValidations: integer;
  ct:integer;
  c_cnodes:integer;
  init:array[0..NUMBER_ROBOTS-1] of integer;
  ind_robo: integer;
  id_rob:integer;
  a_c_flaw:array[0..NUMBER_ROBOTS-1] of integer;
  coms_count:array[0..NUMBER_ROBOTS-1] of integer;
  pre_coms_count:array[0..NUMBER_ROBOTS-1] of integer;
  robots_flaws:array [0..NUMBER_ROBOTS-1] of  Coms_flaws_per_robot;
  t1,t2,tick_p,s_time:QWord;
  total_seconds:longint;
  timestamp_coms:array[0..NUMBER_ROBOTS-1] of longint;
  type_of_movement:array[0..NUMBER_ROBOTS-1] of integer;
  current_step:array[0..NUMBER_ROBOTS-1] of integer;
  current_step_f:array[0..NUMBER_ROBOTS-1] of integer;
   current_step_s:array[0..NUMBER_ROBOTS-1] of integer;
  step_complete:array[0..NUMBER_ROBOTS-1] of integer;
  ghost_nodes:array[0..NUMBER_ROBOTS-1] of integer;
  i_end:array[0..NUMBER_ROBOTS-1] of integer;
  f_rest:array[0..NUMBER_ROBOTS-1] of integer;
  aux_exit_flaw_node:integer;
  f_replan:integer;
  b_pathsend:integer;
  unflawed_node:array of integer;
  flaw_location:coms_flaw_location;
implementation
 uses
   unit1,unit2;
{$R *.lfm}

{ Functions/Procedures }
function round2(const Number: extended; const Places: longint): extended;
var t: extended;
begin
  //Rounds a float value to X decimal points
   t := power(10, places);
   round2 := round(Number*t)/t;
end;
function check_array(ar:i_array;id:integer):integer;
var
s1,aux1,r:integer;
begin
   //Checks if the given integer is part of the array
    s1:=length(ar);
    r:=0;
    if s1>0 then begin
      for aux1:=0 to s1-1 do begin
         if ar[aux1]=id then begin
           r:=1
         end;
      end;
    end;
    check_array:=r;
end;
function blocked_node(var n1:integer; n2:integer):integer;
var
  n_id,ntl:integer;
  l1,l2:integer;
  aux1,aux2:integer;
  c:integer;
begin
   //Verifies if the node N1 is linked to N2 returns 1 if it is
   l1:=length(form1.full_nodelist);
   c:=0;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n2 then
     begin
       l2:=length(form1.full_nodelist[aux1].links);
       for aux2:=0 to l2-1 do
       begin
         ntl:=form1.full_nodelist[aux1].links[aux2].node_to_link;
         if ntl=n1 then
         begin
             c:=1;
         end;
       end;
     end;
   end;
   blocked_node:=c;
end;

function getXcoord(n1:integer):Double;
var
l1:integer;
aux1,n_id:integer;
c:double;
begin
//Obtains the X coord of a given N1 node
 l1:=length(form1.full_nodelist);
   c:=999999999;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n1 then
     begin
      c:=form1.full_nodelist[aux1].pos_X;
     end;
   end;
   getXcoord:=c;
end;

function getYcoord(n1:integer):Double;
var
l1:integer;
aux1,n_id:integer;
c:double;
begin
//Obtains the Y coord of a given N1 node
 l1:=length(form1.full_nodelist);
   c:=999999999;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n1 then
     begin
      c:=form1.full_nodelist[aux1].pos_Y;

     end;
   end;
   getYcoord:=c;
end;

function  get_futhrest_link_node(map:TAStarMap;agvs:r_node;id:integer;CaminhosAgvs:Caminhos;s:integer;unflawed_nodes:i_array):integer;
var
  nid,lid,r,s1,s2,aux1,aux2:integer;
  //x_r,y_r,x_n,y_n,diff_max,diffx,diffy,diff:Double;
begin
   //Returns the a possible entry node to a communication fault, when that fault is detected in the middle of a link
    nid:=agvs[id].inicial_node;
    //x_r:=agvs[id].pos_X;
    //y_r:=agvs[id].pos_y;
    s1:=length(map.TEA_GRAPH);
    //diff_max:=0;
    r:=0;
    for aux1:=0 to s1-1 do begin
       if map.TEA_GRAPH[aux1][0].id=nid then begin
          s2:=length(map.TEA_GRAPH[aux1][0].links);
          for aux2:=0 to s2-1 do begin
            lid:=map.TEA_GRAPH[aux1][0].links[aux2].node_to_link;
            if ((CaminhosAgvs[id].coords[s].node<>lid) and (check_array(unflawed_nodes,lid)=1))then begin
                r:=lid;
                break;
            end;
          end;
          break;
       end;
    end;
 get_futhrest_link_node:=r;
end;

function get_futhrest_exit_link_node(map:TAStarMap;agvs:r_node;id:integer;CaminhosAgvs:Caminhos;se:integer;unflawed_nodes:i_array):integer;
var
  nid,lid,r,s,s1,s2,s3,aux1,aux2,aux3:integer;
 // x_r,y_r,x_n,y_n,diff_max,diffx,diffy,diff:Double;

begin
    //Returns the a possible exit node to a communication fault, when communication is reestablished in the middle of a link
    nid:=agvs[id].inicial_node;
   // x_r:=agvs[id].pos_X;
   // y_r:=agvs[id].pos_y;
    s1:=length(map.TEA_GRAPH);
    // diff_max:=0;
    r:=0;
    s3:=CaminhosAgvs[id].steps;
    for aux1:=0 to s1-1 do begin
       if map.TEA_GRAPH[aux1][0].id=nid then begin
          s2:=length(map.TEA_GRAPH[aux1][0].links);
          for aux2:=0 to s2-1 do begin
            lid:=map.TEA_GRAPH[aux1][0].links[aux2].node_to_link;
          for aux3:=se+1 to s3 do begin
            if ((CaminhosAgvs[id].coords[aux3].node=lid)) then begin
                s:=lid;
                break;
            end;
          end;
       end;
       break;
    end;
    end;

    for aux1:=0 to s1-1 do begin
           if map.TEA_GRAPH[aux1][0].id=nid then begin
              s2:=length(map.TEA_GRAPH[aux1][0].links);
              for aux2:=0 to s2-1 do begin
                lid:=map.TEA_GRAPH[aux1][0].links[aux2].node_to_link;
                if s<>lid then begin
                    r:=lid;
                    break;
                end;
              end;
              break;
           end;
        end;

 get_futhrest_exit_link_node:=r;
end;

function getmaxid(map:TAStarMap):integer;
var
aux1:integer;
id,id_m,l1:integer;
begin
 //Returns the highest Node id number present in the TAStarMap data structure
  l1:=length(map.TEA_GRAPH);
  id_m:=0;
  for aux1:=0 to l1-1 do
  begin
    id:=map.TEA_GRAPH[aux1][0].id;
    if id>id_m then
    begin
      id_m:=id;
    end;
  end;
    getmaxid:=id_m;
end;

function getmaxlid(map:TAStarMap):integer;
var
aux1,aux2:integer;
lid,lid_m,l1,l2:integer;
begin
  //Returns the highest Link id number present in the TAStarMap data structure
  l1:=length(map.TEA_GRAPH);
  lid_m:=0;
  for aux1:=0 to l1-1 do
  begin
    l2:=length(map.TEA_GRAPH[aux1][0].links);
    //id:=map.TEA_GRAPH[aux1][0].id;
     for aux2:=0 to l2-1 do
     begin
         lid:=map.TEA_GRAPH[aux1][0].links[aux2].id_l;
         if lid>lid_m then
         begin
           lid_m:=lid;
         end;
     end;
  end;
    getmaxlid:=lid_m;
end;
function getdisttonode (map:TAStarMap; i:integer; x:double; y:double):Double;
var
l1,aux1,id1:integer;
dist,aux4,aux2,aux3,d_x,d_y,x1,y1:double;
begin
  //Calculates the euclidian distance between the position defined by the X and Y variables to the node with the id i
  l1:=length(form1.map.TEA_GRAPH);
  dist:=99999999999;
  for aux1:=0 to l1-1 do
  begin
    id1:=form1.map.TEA_GRAPH[aux1][0].id;
    if id1=i then begin
      x1:= form1.map.TEA_GRAPH[aux1][0].pos_X;
      y1:= form1.map.TEA_GRAPH[aux1][0].pos_Y;
      d_x:=x-x1;
      d_y:=y-y1;
      aux2:=d_x*d_x;
      aux3:=d_y*d_y;
      aux4:=aux2+aux3;
      dist:=sqrt(aux4);
    end;
  end;
   getdisttonode:=dist;
end;



function create_temp_node(id1:integer; id2:integer; X:Double; Y:Double; r:integer):integer;
var
  s,l1,l2:integer;
  id_aux,idl_aux1,idl_aux2:integer;
  map_aux:TAStarMap;
begin
  //Creates a temporary node to represent the position of a robot when it is travelling in the middle of a link
  l1:=length(form1.map.TEA_GRAPH);
 // map_aux:=form1.map;
  setlength(form1.map.TEA_GRAPH, l1+1,NUM_LAYERS);
  setlength(form1.map.GraphState, l1+1,NUM_LAYERS);
  setlength(form1.map.HeapArray.data, (l1+1)*NUM_LAYERS);
  //form1.map:=map_aux;
  l2:=length(form1.map.TEA_GRAPH);
  id_aux:=getmaxid(form1.map)+1;
  idl_aux1:=getmaxlid(form1.map)+1;
  idl_aux2:=idl_aux1+1;
  for s:=0 to 1 do
  begin
      form1.map.TEA_GRAPH[l1][s].id:=id_aux;
      form1.map.TEA_GRAPH[l1][s].pos_X:=X;
      form1.map.TEA_GRAPH[l1][s].pos_Y:=Y;
      setlength(form1.map.TEA_GRAPH[l1][s].links,2);
      form1.map.TEA_GRAPH[l1][s].links[0].id_l:=idl_aux1;
      form1.map.TEA_GRAPH[l1][s].links[0].node_to_link:=id1;
      form1.map.TEA_GRAPH[l1][s].links[0].distance:=getdisttonode(form1.map,id1,X,Y);
      form1.map.TEA_GRAPH[l1][s].links[1].id_l:=idl_aux2;
      form1.map.TEA_GRAPH[l1][s].links[1].node_to_link:=id2;
      form1.map.TEA_GRAPH[l1][s].links[1].distance:=getdisttonode(form1.map,id2,X,Y);
      form1.map.GraphState[l1][s]:=VIRGIN;
  end;
  c_cnodes:=c_cnodes+1;
  ghost_nodes[r]:=id_aux;
  create_temp_node:=id_aux;
end;

function get_closest_node_id (nodelist:a_node; x:Double; y:Double; scale:integer):integer;

var
    l4:integer;
    aux4:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    id_min:integer;
    diff1:Double;
    diff2:Double;
    Difft:Double;
    diff_min:Double;
begin
  //Returns the id of the node that is closest to the given position, the scale integer is used when the position is not given in cartesian coordinates
  l4:=length(nodelist);
  diff_min:=10*scale;
  id_min:=0;
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     x_p:=nodelist[aux4].pos_X*scale;
     y_p:=nodelist[aux4].pos_Y*scale;
     diff1:=abs(x_p-x*scale);
     diff2:=abs(y_p-y*scale);
     Difft:=diff1+diff2;
  if diff_min>Difft then
     begin
      diff_min:=Difft;
      id_min:=n_curr;
    end;
 end;
end;
  get_closest_node_id:=id_min;
end;
function getdirlink (dx:double; dy:double):integer;
begin
  //Determines in which the direction the robot is travellling and represents it via a integer
  if ((dx = 0) and (dy >0)) then begin getdirlink := 0; end
  else if ((dx = 0) and (dy <0)) then begin getdirlink := 4; end
  else if ((dx > 0) and (dy = 0)) then begin getdirlink := 2; end
  else if ((dx <0) and (dy = 0)) then begin getdirlink := 6; end
  else if ((dx >0) and (dy >0)) then begin getdirlink := 1; end
  else if ((dx >0) and (dy <0)) then begin getdirlink := 3; end
  else if ((dx <0) and (dy >0)) then begin getdirlink := 7; end
  else if ((dx <0) and (dy <0)) then begin getdirlink := 5; end;
end;


function round_with_th(a1:double;a2:double;td:double):double;
var
r:double;
begin
  //Rounds a number based on a threshold (currently not used)
    if (((a2-td)<=a1) and ((a2+td)>=a1)) then begin
    r:=a2;
    end else begin
    r:=a1;
    end;
    //round_with_th:=r;
end;


function get_linked_node(nodelist:a_node; x:Double; y:Double; scale:integer; id1:integer):integer;
var
    l4,l2,aux1,aux2,aux3,n_curr,id2_aux,id2:integer;
    x1,y1,x2,y2:double;
    diff1x,diff1y,diff2x,diff2Y:double;
    dir1,dir2:integer;
begin
  //Returns the id of the node that is also part of the link, unlike the previous function this one does not require prior knowledge of the node that is currently
  //atributed to the robot. This function uses the coordinates of the referencial in order to get an approximation of the robot current location inside the graph
  l4:=length(nodelist);
  id2:=0;
   for aux1:=0 to l4-1 do
   begin
     n_curr:=nodelist[aux1].id;
     if n_curr=id1 then
        begin
        l2:=length(nodelist[aux1].links);
        x1:=nodelist[aux1].pos_X;
        x1:=round_with_th(x1,x,0.05);
        y1:=nodelist[aux1].pos_Y;
        y1:=round_with_th(y1,y,0.05);
        diff1x:=(x-x1);
        diff1y:=(y-y1);
        dir1:=getdirlink(diff1x,diff1y);
        for aux2:=0 to l2-1 do
        begin
           id2_aux:=nodelist[aux1].links[aux2].node_to_link;
           for aux3:=0 to l4-1 do
           begin
             if id2_aux=nodelist[aux3].id then begin
              x2:=nodelist[aux3].pos_X;
             // x2:=round_with_th(x2,x1,0.0.1);
              y2:=nodelist[aux3].pos_Y;
              //y2:=round_with_th(y2,y1,0.1);
             end;
           end;
            diff2x:=(x2-x1);
            diff2y:=(y2-y1);
            dir2:=getdirlink(diff2x,diff2y);
            if dir2=dir1 then
               begin
                id2:=id2_aux
               end;
        end;
   end;
   end;
   get_linked_node:=id2;
end;
 procedure delete_ghost_node(i:integer);
 var
     id_aux1,aux3,id_aux2,s4,s3,s2,s1,aux2,aux1:integer;
     map_aux:TAStarMap;
 begin
    //This function deletes the nodes created in order to represent the position of the robots when they are travelling in the middle of a link
      s2:=length((form1.map.TEA_GRAPH));
      for aux1:=0 to length(form1.map.TEA_GRAPH)-1 do
      begin
       if form1.map.TEA_GRAPH[aux1][0].id=i then begin
            for aux2:=aux1 to length(form1.map.TEA_GRAPH)-2 do begin
                for s1:=0 to 1 do begin
                    id_aux1:=form1.map.TEA_GRAPH[aux2][s1].id;
                    id_aux2:=form1.map.TEA_GRAPH[aux2+1][s1].id;
                    form1.map.TEA_GRAPH[aux2][s1]:=form1.map.TEA_GRAPH[aux2+1][s1];
                    form1.map.GraphState[aux2][s1]:= form1.map.GraphState[aux2+1][s1];
                    form1.map.TEA_GRAPH[aux2][s1].id:=id_aux1;
                    s4:=length(form1.robots);
                    for aux3:=0 to s4-1 do begin
                        if form1.robots[aux3].inicial_node=id_aux2 then begin
                           form1.robots[aux3].inicial_node:=id_aux1;
                           ghost_nodes[aux3]:=id_aux1;
                        end;
                    end;
                end;
            end;
              //map_aux:=form1.map;
              setlength(form1.map.TEA_GRAPH, length(form1.map.TEA_GRAPH)-1,NUM_LAYERS);
              setlength(form1.map.GraphState, length(form1.map.TEA_GRAPH),NUM_LAYERS);
              setlength(form1.map.HeapArray.data, (length(form1.map.TEA_GRAPH))*NUM_LAYERS);
              s3:=length((form1.map.TEA_GRAPH));
              break;
              //form1.map:=map_aux;
        end;
      end;
 end;
  function remove_from_unflawed_node(ind:integer;unflawed_nodes:i_array):i_array;
  var
      s1,aux1:integer;
      aux_r:i_array;
  begin
    //Removes a node from the array of unflawed nodes
    s1:=length(unflawed_nodes);
    aux_r:=unflawed_nodes;
   for aux1:=ind to s1-2 do begin
       aux_r[aux1]:=aux_r[aux1+1];
   end;
   setlength(aux_r,s1-1);
   remove_from_unflawed_node:=aux_r;
  end;

 function check_unflawed_node(id:i_array;unflawed_nodes:i_array):i_array;
 var
 s1,s2,aux2,aux1:integer;
 aux_r:i_array;
 begin
   //Checks the array of unflawed nodes  looking for a specific nodes
   s1:=length(unflawed_nodes);
   s2:=length(id);
   aux_r:=unflawed_nodes;
   for aux2:=0 to s2-1 do begin
   for aux1:=0 to s1-1 do begin
       if unflawed_nodes[aux1]=id[aux2] then begin
            aux_r:=remove_from_unflawed_node(aux1,unflawed_nodes);
       end;
   end;
   end;
   check_unflawed_node:=aux_r;
 end;

 function remove_flaw(ind:integer;robots_flaws_l:coms_flaw_location):coms_flaw_location;
 var
 aux2,s2,s1,aux1:integer;
 aux_r:coms_flaw_location;
 begin
    //Cleans the data structures that store the data relative to an specific Communication fault
    aux_r:=robots_flaws_l;
    s1:=length(robots_flaws_l.detected_nodes);
   for aux1:=ind to s1-2 do begin
     aux_r.in_node[aux1]:=aux_r.in_node[aux1+1];
     aux_r.unvin_node[aux1]:=aux_r.unvin_node[aux1+1];
     aux_r.detected_nodes[aux1]:=aux_r.detected_nodes[aux1+1];
   end;
   setlength(aux_r.in_node,s1-1);
   setlength(aux_r.unvin_node,s1-1);
   setlength(aux_r.detected_nodes,s1-1);
   s2:=length(robots_flaws);
   for aux2:=0 to s2-1 do begin
     if robots_flaws[aux2].flaw_ind>ind then begin
         robots_flaws[aux2].flaw_ind:=robots_flaws[aux2].flaw_ind-1;
     end else if robots_flaws[aux2].flaw_ind=ind then begin
         robots_flaws[aux2].flaw_ind:=99999;
     end;
   end;
   remove_flaw:=aux_r;
 end;


 function check_if_flaw_ceased(id:integer;robots_flaws_l:coms_flaw_location):coms_flaw_location;
 var
 at,s2,aux2,s1,aux1:integer;
 aux_r:coms_flaw_location;
 begin
   //Verifies if a certain communication fault as ceased to affect the system
   at:=0;
   s1:=length(robots_flaws_l.in_node);
   aux_r:=robots_flaws_l;
   for aux1:=0 to s1-1 do begin
     s2:=length(robots_flaws_l.in_node);
     for aux2:=0 to s2-1 do begin
        if id=robots_flaws_l.in_node[aux1][aux2] then begin
          at:=1;
        end;
     end;
   end;
 //if at=0 then begin
 //  s1:=length(robots_flaws_l.out_node);
 //  for aux1:=0 to s1-1 do begin
 //    s2:=length(robots_flaws_l.out_node);
 //    for aux2:=0 to s2-1 do begin
 //      if id=robots_flaws_l.out_node[aux1][aux2] then begin
 //         at:=1;
 //       end;
 //    end;
 //  end;
 //end;

 if at=0 then begin
   s1:=length(robots_flaws_l.detected_nodes);
   for aux1:=0 to s1-1 do begin
     s2:=length(robots_flaws_l.detected_nodes);
     for aux2:=0 to s2-1 do begin
        if robots_flaws_l.detected_nodes[aux1][aux2]=id then begin
          aux_r:=remove_flaw(aux1,robots_flaws_l);
        end;
     end;
   end;
 end;
 check_if_flaw_ceased:=aux_r;
 end;
function updaterobotnode(nodelist:a_node; x:Double; y:Double; scale:integer; r:integer):integer;
var
    aux1,l4,s1,idf:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    threshold_d:Double;
    id1_aux,id2_aux:integer;
begin
  //Updates the robot position
  l4:=length(nodelist);
  idf:=0;
  threshold_d:=0.08;
  if ghost_nodes[r]<>0 then begin
     delete_ghost_node(ghost_nodes[r]);
  end;
  for aux1:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux1].id;
     x_p:=nodelist[aux1].pos_X*scale;
     y_p:=nodelist[aux1].pos_Y*scale;
    if (((x_p-threshold_d<=x) and(x<=x_p+threshold_d)) and ((y_p-threshold_d<=y) and(y<=y_p+threshold_d))) then
        begin
         idf:=n_curr;
        end;
  end;
  if idf=0 then
     begin
       id1_aux:=get_closest_node_id(nodelist,x,y,scale);
       id2_aux:=get_linked_node(nodelist,x,y,scale,id1_aux);
       if id2_aux=0 then begin
          id2_aux:=id1_aux;
       end;
       idf:=create_temp_node(id1_aux,id2_aux,x,y,r);
      ghost_nodes[r]:=idf;
     end  else begin
        if check_array( unflawed_node,idf)=0 then begin
        s1:=length( unflawed_node);
        setlength( unflawed_node,s1+1);
        unflawed_node[s1]:=idf;
        flaw_location:=check_if_flaw_ceased(idf,flaw_location);
        end;
         ghost_nodes[r]:=0;
     end;
 updaterobotnode:=idf;
end;
function get_remaining_path_nodes(CaminhosAgvs:Caminhos; i:integer;n:integer;unflawed_nodes:i_array;robot:cf_robots;s:integer):i_array;
var
 s1,s2,aux1,aux2,at,c_a:integer;
 r_aux:array of integer;
begin
 s1:=CaminhosAgvs[i].steps;
 at:=0;
 i_end[i]:=0;
 // s1:=length(CaminhosAgvs[i].coords);
        for aux2:=s to s1 do begin
          if ((check_array(unflawed_nodes,CaminhosAgvs[i].coords[aux2].node)=1) and (aux2<>s)) then begin
          s2:=length(r_aux);
          //setlength(r_aux,s2+1);
          //r_aux[s2]:= CaminhosAgvs[i].coords[aux2].node;
          break;
          end else if  (CaminhosAgvs[i].coords[aux2].node<>0) then begin
             s2:=length(r_aux);
             setlength(r_aux,s2+1);
             r_aux[s2]:= CaminhosAgvs[i].coords[aux2].node;
             if CaminhosAgvs[i].coords[aux2].node=n then begin
                  at:=1
             end;
           end;
        end;
  get_remaining_path_nodes:=r_aux;
end;
function update_flaw_in_node(CaminhosAgvs:Caminhos; i:integer;n:integer;unflawed_nodes:i_array;d_node:i_array; robots:cf_robots; s:integer;in_node:i_array):i_array;
var
   s1,s2,aux1:integer;
  a_ar,d_ar:i_array;
begin
  //This function ajust the dimensions of the detected communication fault
   a_ar:=get_remaining_path_nodes(CaminhosAgvs,i,n,unflawed_nodes,robots,s);
   d_ar:=d_node;
   s1:=length(a_ar);
   for aux1:=0 to s1-1 do begin
    if ((check_array(d_ar,a_ar[aux1])=1) or (check_array(in_node,a_ar[aux1])=1)) then begin
    end else begin
        s2:=length(d_ar);
        setlength(d_ar,s2+1);
        d_ar[s2]:=a_ar[aux1]
    end;
   end;
   update_flaw_in_node:=d_ar;
end;
function check_if_c_nodes_already_detected (aux_i:i_array;robots_flaws:coms_flaw_location;p:integer):integer;
var
r,s1,s2,s3,s4,aux4,aux3,aux1,aux2,c_obj,c:integer;
begin
  //This function checks if the nodes that were detected as having communication faults are already associated with  an existing fault
  r:=0;
  s1:=length(robots_flaws.detected_nodes);
  s3:=length(aux_i);
  c_obj:=round(S3/p);
  if c_obj=0 then begin
  c_obj:=1;
  end;
  for aux1:=0 to s1-1 do begin
    s2:=length(robots_flaws.detected_nodes[aux1]);
    c:=0;
    for aux2:=0 to s2-1 do begin
      for aux3:=0 to s3-1 do begin
       s4:=length(robots_flaws.unvin_node[aux1][aux2]);
        if robots_flaws.detected_nodes[aux1][aux2]= aux_i[aux3] then begin
           c:=c+1;
        end;
        for aux4:=0 to s4-1 do begin
            if robots_flaws.unvin_node[aux1][aux2][aux4]= aux_i[aux3] then begin
               c:=c+1;
            end;
        end;
        if c>=c_obj then begin
          r:=aux1+1;
          break;
        end;
      end;
    end;
  end;
  check_if_c_nodes_already_detected:=r;
end;
function getlinkeden_nodes(map:TAStarMap;f_nodes:i_array;i_node:i_array):ii_array;
var
s1,s2,s3,s4,lid,aux1,aux2,aux3,auxid:integer;
r:array of array of integer;
begin
  //This function transforms all the unfaulted that are directly connected to a fault node as entry/exit nodes of that fault
  s1:=length(f_nodes);
  s2:=length(map.TEA_GRAPH);
  setlength(r,s1);
  for aux1:=0 to s1-1 do begin
      auxid:=f_nodes[aux1];
   for aux2:=0 to s2-1 do begin
       if map.TEA_GRAPH[aux2][0].id=auxid then begin
           s3:=length(map.TEA_GRAPH[aux2][0].links);
          for aux3:=0 to s3-1 do begin
            lid:=map.TEA_GRAPH[aux2][0].links[aux3].node_to_link;
            if ((check_array(i_node,lid)<>1) and (check_array(f_nodes,lid)<>1)) then begin
               s4:=length(r[aux1]);
               setlength(r[aux1],s4+1);
               r[aux1][s4]:=lid;
            end;
          end;
       end;
   end;
  end;
  getlinkeden_nodes:=r;
end;

function save_flaw_location(map:TAStarMap;CaminhosAgvs:Caminhos; agvs:r_node; i:integer; nodelist:a_node; s:integer;robots_flaws_l:coms_flaw_location;robots_flaw_status:cf_robots;unflawed_nodes:i_array):coms_flaw_location;
var
   n_curr, at,idf,aux1,aux2,aux_id,s1,s2,s3,s4:integer;
   aux_robots_flaws: coms_flaw_location;
   aux_i:i_array;
   ind:integer;
begin
   //This function regists every time a fault is detected, it is responsible for either storing the new fault or assosiating the detected nodes to a pre-existing  fault
   aux_robots_flaws:=robots_flaws_l;
   robots_flaw_status[i].isactive:=1;
   robots_flaw_status[i].active_consecutive_hits:=1;
   s4:=length(nodelist);
   idf:=0;
   for aux1:=0 to s4-1 do
   begin
     n_curr:=nodelist[aux1].id;
     if n_curr=agvs[i].inicial_node then
        begin
         idf:=n_curr;
        end;
  end;
   if idf=0 then begin
   //aux_id:=get_linked_node(nodelist,agvs[i].pos_X,agvs[i].pos_y,1,CaminhosAgvs[i].coords[s].node);
    aux_id:=get_futhrest_link_node(form1.map,form1.robots,i,CaminhosAgvs,s,unflawed_nodes);
   end else begin
    aux_id:=idf;
   end;
   s1:=length(aux_robots_flaws.in_node);
   at:=0;
   robots_flaw_status[i].curr_in_node:=aux_id;
   if s1>0 then begin
   for aux1:=0 to s1-1 do begin
     s2:=length(aux_robots_flaws.in_node[aux1]);
      for aux2:=0 to s2-1 do begin
       if aux_robots_flaws.in_node[aux1][aux2]=aux_id then begin
         at:=aux1+1;
         //aux_robots_flaws.flaw_ind:=aux1;
       end;
      end;
   end;
   end;
   //if at=0 then begin
   //s2:=length(aux_robots_flaws.out_node);
   //if s2>0 then begin
   //for aux1:=0 to s2-1 do begin
   //  s3:=length(aux_robots_flaws.out_node[aux1]);
   //  for aux2:=0 to s3-1 do begin
   //    if aux_robots_flaws.out_node[aux1][aux2]=aux_id then begin
   //      at:=aux1+1;
   //    end;
   //end;
   //end;
   //end;
   //end;

   if at=0 then begin
      aux_i:=get_remaining_path_nodes(CaminhosAgvs,i,aux_id,unflawed_nodes,robots_flaw_status,s);
      if check_if_c_nodes_already_detected(aux_i,robots_flaws_l,1)=0 then begin
      setlength(aux_robots_flaws.in_node, s1+1);
      setlength(aux_robots_flaws.in_node[s1], 1);
      aux_robots_flaws.in_node[s1][0]:=aux_id;
      robots_flaw_status[i].isdetecting:=1;
      aux_robots_flaws.n_flaws:= aux_robots_flaws.n_flaws+1;
      s2:=length(aux_robots_flaws.detected_nodes);
      setlength(aux_robots_flaws.detected_nodes,s1+1);
      aux_robots_flaws.detected_nodes[s1]:=get_remaining_path_nodes(CaminhosAgvs,i,aux_id,unflawed_nodes,robots_flaw_status,s);
      setlength(aux_robots_flaws.unvin_node,s1+1);
      aux_robots_flaws.unvin_node[s1]:=getlinkeden_nodes(map,aux_robots_flaws.detected_nodes[s1],aux_robots_flaws.in_node[s1]);
      unflawed_nodes:=check_unflawed_node( aux_robots_flaws.detected_nodes[s1],unflawed_nodes);
      robots_flaw_status[i].flaw_ind:=s1;
      end else begin
         ind:=check_if_c_nodes_already_detected(aux_i,robots_flaws_l,2);
         ind:=ind-1;
         s3:=length(aux_robots_flaws.in_node[ind]);
         setlength(aux_robots_flaws.in_node[ind], s3+1);
         aux_robots_flaws.in_node[ind][s3]:=aux_id;
         robots_flaw_status[i].isdetecting:=0;
         aux_robots_flaws.detected_nodes[ind]:=update_flaw_in_node(CaminhosAgvs,i,aux_id,unflawed_nodes, aux_robots_flaws.detected_nodes[ind],robots_flaw_status,s,aux_robots_flaws.in_node[ind]);
         aux_robots_flaws.unvin_node[ind]:=getlinkeden_nodes(map,aux_robots_flaws.detected_nodes[ind],aux_robots_flaws.in_node[ind]);
         unflawed_nodes:=check_unflawed_node(aux_robots_flaws.detected_nodes[ind],unflawed_nodes);
         robots_flaw_status[i].flaw_ind:=ind;
      end;
   end else begin
      ind:=at;
      ind:=ind-1;
      s3:=length(aux_robots_flaws.in_node[ind]);
      setlength(aux_robots_flaws.in_node[ind], s3+1);
      aux_robots_flaws.in_node[ind][s3]:=aux_id;
      robots_flaw_status[i].isdetecting:=0;
      aux_robots_flaws.detected_nodes[ind]:=update_flaw_in_node(CaminhosAgvs,i,aux_id,unflawed_nodes, aux_robots_flaws.detected_nodes[ind],robots_flaw_status,s,aux_robots_flaws.in_node[ind]);
      aux_robots_flaws.unvin_node[ind]:=getlinkeden_nodes(map,aux_robots_flaws.detected_nodes[ind],aux_robots_flaws.in_node[ind]);
      unflawed_nodes:=check_unflawed_node(aux_robots_flaws.detected_nodes[ind],unflawed_nodes);
      robots_flaw_status[i].flaw_ind:=ind;
   end;
    CaminhosAgvs_af[i]:=CaminhosAgvs[i];
    robots_flaws[i]:=robots_flaw_status[i];
   save_flaw_location:=aux_robots_flaws;
end;


function update_flaw_path(n:integer;ni:integer;f_ar:i_array ; i:integer; hitcount:integer):i_array;
var
 at,s1,s2,aux1,aux2,c_a,a1,a2,c1,c2,diff:integer;
 r_aux:array of integer;
begin
//This function ajust the dimensions of the detected communication fault by removing nodes from the fault when communication can be estableshied with them
 s1:=length(f_ar);
 at:=0;
 c_a:=0;
 if ((n=ni) and (hitcount>1)) then begin
 for aux1:=0 to s1-1 do begin
     if ((f_ar[aux1]=n) and (at=0)) then begin
         at:=1;
     end else if ((f_ar[aux1]=n) and (at=1)) then begin
           c_a:=aux1+1;
           break;
       end;
     end;
 end else begin
        for aux1:=0 to s1-1 do begin
           if ((f_ar[aux1]=n)) then begin
                 c_a:=aux1+1;
                 break;
             end;
         end;
end;
if c_a=0 then begin
c_a:=s1-1
end else begin
   c_a:=c_a-1;
end;

for aux2:=0 to c_a do begin
    s2:=length(r_aux);
    setlength(r_aux,s2+1);
    r_aux[s2]:=f_ar[aux2];
end;
if i_end[i]=1 then  begin
   s2:=length(r_aux);
   setlength(r_aux,s2+1);
   r_aux[s2]:=f_ar[s1-1];
end;
update_flaw_path:=r_aux;
end;



 {
  s1:=length(CaminhosAgvs[i].coords);
  c1:=99999;
  c2:=99999;
  for aux1:=0 to s1-1 do begin
     if ((CaminhosAgvs[i].coords[aux1].node=ni) and (c1=99999)) then begin
       c1:=aux1;
     end else if ((CaminhosAgvs[i].coords[aux1].node=n) and (c2=99999)) then begin
       c2:=aux1;
     end;
  end;
  if c1 > c2 then begin
    diff:=c2-c1;
  end;
  diff:=c2-c1;
  setlength(r_aux,diff+1);
  c_a:=0;
  if diff=0 then begin
  s2:=length(form1.crit_nodes);
  at:=0;
  for aux1:=0 to s2-1 do begin
  if form1.crit_nodes[aux1]=n then begin
    at:=1
  end else begin
      at:=0;
  end;
  end;
  if at=1 then begin
        a1:=99999;
       a2:=99999;
     for aux2:=0 to s1-1 do begin
     if ((CaminhosAgvs[i].coords[aux1].node=n) and (a1=99999)) then begin
       a1:=aux1;
     end else if ((CaminhosAgvs[i].coords[aux1].node=n) and (a2=99999) and (a2<>a1)) then begin
       a2:=aux1;
     end;
     end;
     diff:=a2-a1;
     setlength(r_aux,diff+1);
     c_a:=0;
    for aux2:=a1 to a2 do begin
       r_aux[c_a]:= CaminhosAgvs[i].coords[aux2].node;
       c_a:=c_a+1;
    end;
  end else begin
    r_aux[c_a]:= CaminhosAgvs[i].coords[c2].node;
  end;
  end else begin
  for aux2:=c1 to c2 do begin
     r_aux[c_a]:= CaminhosAgvs[i].coords[aux2].node;
     c_a:=c_a+1;
  end;
  end;
}

function save_exit_node(map:TAStarMap;agvs:r_node;i:integer;robots_flaws:cf_robots;coms_f_l:coms_flaw_location;unflawed_nodes:i_array):coms_flaw_location;
var
    aux_robots_flaws: coms_flaw_location;
    aux_id,aux1,s1,s2,s4,n_curr,idf:integer;
begin
   //Regists the exit node of a certain fault and updates the dimension of the fault
   aux_robots_flaws:=coms_f_l;
   s1:=robots_flaws[i].flaw_ind;
   s2:=length(aux_robots_flaws.in_node[s1]);
   setlength(aux_robots_flaws.in_node[s1],s2+1);
   s4:=length(form1.full_nodelist);
   idf:=0;
   for aux1:=0 to s4-1 do
   begin
     n_curr:=form1.full_nodelist[aux1].id;
     if n_curr=agvs[i].inicial_node then
        begin
         idf:=n_curr;
        end;
  end;
   if idf=0 then begin
   aux_id:=get_futhrest_exit_link_node(form1.map,form1.robots,i,CaminhosAgvs_af,current_step_f[i],unflawed_nodes);
   end else begin
    aux_id:=idf;
   end;
   robots_flaws[i].curr_out_node:=aux_id;
   aux_robots_flaws.in_node[s1][s2]:=aux_id;
   aux_robots_flaws.detected_nodes[s1]:=update_flaw_path(robots_flaws[i].curr_out_node,robots_flaws[i].curr_in_node,aux_robots_flaws.detected_nodes[s1],i,robots_flaws[i].active_consecutive_hits);
   aux_robots_flaws.unvin_node[s1]:=getlinkeden_nodes(map,aux_robots_flaws.detected_nodes[s1],aux_robots_flaws.in_node[s1]);
   robots_flaws[i].active_consecutive_hits:=0;
   save_exit_node:=aux_robots_flaws;
end;

function update_afected_nodes(coms_f_l:coms_flaw_location;robots_flaws:cf_robots;i:integer;unflawed_nodes:i_array;s:integer):coms_flaw_location;
var
    s3,s1,s2,aux1,aux2,c_a,c1,c2,diff:integer;
    r_aux:coms_flaw_location;
    a_ar,d_ar:i_array;
begin
    //Regists the occurance of a new fault and makes an initial approximation of it's overall dimention
     r_aux:=coms_f_l;
     s1:=length(r_aux.in_node[robots_flaws[i].flaw_ind]);
     setlength(r_aux.in_node[robots_flaws[i].flaw_ind],s1+1);
     r_aux.in_node[robots_flaws[i].flaw_ind][s1]:=robots_flaws[i].curr_out_node;
     a_ar:=get_remaining_path_nodes(CaminhosAgvs,i,robots_flaws[i].curr_in_node,unflawed_nodes,robots_flaws,s);
     s2:=length(a_ar);
     c1:=0;
     for aux1:=0 to s2-1 do begin
         if a_ar[aux1]=robots_flaws[i].curr_out_node then begin
            c1:=aux1+1;
            break
         end;
     end;
     if c1=0 then begin
     c1:=s2;
     end;
     for aux1:=0 to c1-1 do begin
       s3:=length( r_aux.detected_nodes[robots_flaws[i].flaw_ind]);
       setlength( r_aux.detected_nodes[robots_flaws[i].flaw_ind],s3+1);
       r_aux.detected_nodes[robots_flaws[i].flaw_ind][s3]:=a_ar[aux1];
     end;
     update_afected_nodes:=r_aux;
end;

function  checkfornode (nodelist:a_node; x:Double; y:Double; scale:integer):integer;
 var
    aux1,l4,idf:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    threshold_d:Double;
    id1_aux,id2_aux:integer;
begin
  //Verifies if given X and Y coordinates are within a certain threshold of a node. In the global view it serves to check if the robot is indeed on top
  //of a given node.the threshold values can be ajusted depending on the desired precision
  l4:=length(nodelist);
  idf:=0;
  threshold_d:=0.05;
  for aux1:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux1].id;
     x_p:=nodelist[aux1].pos_X*scale;
     y_p:=nodelist[aux1].pos_Y*scale;
     if (((x_p-threshold_d<=x) and(x<=x_p+threshold_d)) and ((y_p-threshold_d<=y) and(y<=y_p+threshold_d))) then
        begin
         idf:=n_curr;
        end
  end;
  checkfornode:=idf;
end;
procedure removenodes(map:TAStarMap;c:integer);
var
l1:integer;
begin
 //Simplified version of the remove temporary nodes function
 //(only works if the temporary nodes are always indexed at the end of the stack and if the oldest temporary node is the first item of the stack)
 l1:=length(map.TEA_GRAPH);
 setlength(map.TEA_GRAPH,l1-c,NUM_LAYERS);
end;

function get_steps(var CaminhosAgvs:Caminhos;i:integer):integer;
 var
 l1,aux1,aux2:integer;
 Begin
 //Get the number of steps that each robot needs to execute
l1:=length((CaminhosAgvs[i].coords));
aux2:=0;
for aux1:=0 to l1-1 do begin
if ((getXcoord(CaminhosAgvs[i].coords[aux1].node)<3) and (getYcoord(CaminhosAgvs[i].coords[aux1].node)<3)) then
           begin
           aux2:=aux2+1
           end;
   end;
get_steps:=aux2;
end;

procedure InitialPointsForAllRobots(var agvs:r_node);
var
  v:integer;
begin
    //Inicialize all the points for the robots in the simtwo simulator
    MessageInitialPositions := '';
    MessageInitialPositions := MessageInitialPositions + 'T' + IntToStr(NUMBER_ROBOTS);
    v:=0;
    while v < NUMBER_ROBOTS do begin
      ListPriorities[v]:=v;
      MessageInitialPositions := MessageInitialPositions + 'R' + 'X' +
                                 floatToStr(round2(agvs[v].pos_X,3)) + 'Y' +
                                 floatToStr(round2(agvs[v].pos_Y,3)) + 'D' +
                                 floatToStr(agvs[v].Direction);
      xDest[v]:=agvs[v].pos_X;
      yDest[v]:=agvs[v].pos_Y;
      v:=v+1;
    end;
    MessageInitialPositions := MessageInitialPositions + 'F';
end;

function checkforpathcompletion(agvs:r_node; i:integer):integer;
var
  l1,aux1,r:integer;
begin
  //checks if the robot as completed its path
  r:=0;
  if agvs[i].inicial_node=agvs[i].target_node then
  begin
   r:=1;
  end;
  checkforpathcompletion:=r;
end;
function return_id_frompriority(var agvs: r_node; p:integer):integer;
var
 l1,aux1,r:integer;
Begin
  //Returns the id of the robot that as the desired priority
  r:=0;
  l1:=length(agvs);
  for aux1:=0 to l1-1 do
  begin
    if p=agvs[aux1].InitialIdPriority then
    begin
         r:=aux1;
    end;
  end;
   return_id_frompriority:=r;
end;

function get_t_mov(CaminhosAgvs:Caminhos; i:integer):m_node;
var
  aux1:integer;
  aux_array:array[0..NUMBER_ROBOTS-1] of integer;
begin
  //Analyse the robot movement and classify it
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
    if CaminhosAgvs[aux1].coords[i-1].node=CaminhosAgvs[aux1].coords[i].node then
    begin
      if CaminhosAgvs[aux1].coords[i-1].direction=CaminhosAgvs[aux1].coords[i].direction then
      begin
         aux_array[aux1]:=1;//wait
      end else begin
         aux_array[aux1]:=2;//rotate
      end;
    end
    else begin
         aux_array[aux1]:=3; //go foward
    end;
  end;
  get_t_mov:=aux_array;
end;
function  check_step_comp(CaminhosAgvs:Caminhos;agvs: r_node;s:m_node): m_node;
var
  aux1:integer;
  diffY,diffx,td,aux_rx,aux_ry,aux_x,aux_y:double;
  res:array[0..NUMBER_ROBOTS-1] of integer;
begin
  //check if the robot as completed it's desired step
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      aux_x:=round2(getXcoord(CaminhosAgvs[aux1].coords[s[aux1]].node),3);
      aux_y:=round2(getycoord(CaminhosAgvs[aux1].coords[s[aux1]].node),3);
      aux_rx:=round2(agvs[aux1].pos_X,3);
      aux_ry:=round2(agvs[aux1].pos_y,3);
      diffx:=abs(aux_x-aux_rx);
      diffY:=abs(aux_y-aux_ry);
      td:=0.02;
      if ((agvs[aux1].inicial_node=CaminhosAgvs[aux1].coords[s[aux1]].node) and (CaminhosAgvs[aux1].coords[s[aux1]].direction=agvs[aux1].Direction) and (agvs[aux1].onrest<>1)) then
      begin
         res[aux1]:=1;
      end else begin
         res[aux1]:=0;
      end;
  end;
   check_step_comp:=res;
end;
function check_if_too_ahead_on_step(CaminhosAgvs:Caminhos;agvs: r_node;s:m_node):integer;
var
min,s1,r,diff,aux1:integer;
x_n,y_n,x_r,y_r,diffx,diffy,difft:double;
begin
  //check if the robot is desyncronized in relation with the others, in this case checks if the robot is too ahead than the remaining
    s1:=length(s);
    min:=9999999;
    r:=0;
    for aux1:=0 to s1-1 do begin
        if ((s[aux1]<=min) and (agvs[aux1].onrest<>1)) then begin
          min:=s[aux1];
       end;
    end;
     for aux1:=0 to s1-1 do begin
       diff:=s[aux1]-min;
       if diff>0 then begin
           x_n:=getXcoord(CaminhosAgvs[aux1].coords[min].node);
           y_n:=getYcoord(CaminhosAgvs[aux1].coords[min].node);
           x_r:=agvs[aux1].pos_X;
           y_r:=agvs[aux1].pos_y;
           diffx:=abs(x_n-x_r);
           diffy:=abs(y_n-y_r);
           difft:=diffx+diffy;
           if difft>=0.2 then begin
             r:=1;
           end;
       end;
     end;
     check_if_too_ahead_on_step:=r;
end;

function check_if_too_close(agvs:r_node):integer;
var
lnode,inode2,inode,onode,aux1,s1,s2,s3,aux2,aux3,aux4,aux5,aux6,aux7,r:integer;
begin
  //check if 2 or more robots are in a situation where a small desyncronisation can lead to a deadlock or colision
  r:=0;
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
   onode:=CaminhosAgvs[aux1].coords[current_step[aux1]].node;
   for aux2:=0 to NUMBER_ROBOTS-1 do begin
    inode:=agvs[aux2].inicial_node;
     if ((ghost_nodes[aux2]=inode) and (aux1<>aux2)) then begin
     for aux3:=0 to length(form1.map.TEA_GRAPH)-1 do
      begin
       if form1.map.TEA_GRAPH[aux3][0].id=inode then begin
               for aux4:=0 to length(form1.map.TEA_GRAPH[aux3][0].links)-1 do
                begin
                   inode2:=form1.map.TEA_GRAPH[aux3][0].links[aux4].node_to_link;
                   if ((onode=inode2)) then begin
                      r:=1;
                      break;
                   end;
                end;
            end;
       end;
      end
    else if ((onode=inode) and (aux1<>aux2)) then begin
    r:=1;
    end;

   end;
end;





  { s1:=length(form1.map.TEA_GRAPH);
  r:=0;
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      inode:=agvs[aux1].inicial_node;
      for aux2:=0 to s1-1 do begin
        if form1.map.TEA_GRAPH[aux2][0].id=inode then begin
            s2:=length(form1.map.TEA_GRAPH[aux2][0].links);
            for aux3:=0 to s2-1 do begin
               lnode:=form1.map.TEA_GRAPH[aux2][0].links[aux3].node_to_link;
               for aux4:=0 to NUMBER_ROBOTS-1 do begin
                  onode:=agvs[aux1].inicial_node;
                   for aux5:=0 to s1-1 do begin
                    if form1.map.TEA_GRAPH[aux2][0].id=onode then begin
                        s3:=length(form1.map.TEA_GRAPH[aux2][0].links);
                        for aux6:=0 to s3-1 do begin
                           lonode:=form1.map.TEA_GRAPH[aux2][0].links[aux3].node_to_link;
                           if lonode=lnode then begin
                               r:=1;
                               break;
                           end;
                        end;
                    end
                   end;
                 if onode>length(form1.full_nodelist) then begin

                 end else begin
                      if  onode=lnode then begin
                          r:=1;
                          break;
                      end;
                 end;
               end;
            end;
        end;
      end;
  end;}
  check_if_too_close:=r;
end;


function check_if_still_on_path(CaminhosAgvs:Caminhos;agvs: r_node;i:integer;t_d:double;s:m_node):integer;
var
tom,step_complete:array[0..NUMBER_ROBOTS-1] of integer;
aux_rx,aux_ry,aux_x,aux_y:double;
r,aux_step_complete:integer;
begin
  //Check if the robot is still executing its path correctly
  tom:=get_t_mov(CaminhosAgvs,1);
  r:=0;
  step_complete:=check_step_comp(CaminhosAgvs,agvs,s);
  aux_step_complete:=0;
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      if step_complete[aux1]=1 then begin
         aux_step_complete:=1;
      end;
  end;
  if NUMBER_ROBOTS>1 THEN BEGIN
  for i:=0 to  NUMBER_ROBOTS-1 do begin
  aux_x:=round2(getXcoord(CaminhosAgvs[i].coords[s[i]].node),3);
  aux_y:=round2(getYcoord(CaminhosAgvs[i].coords[s[i]].node),3);
  aux_rx:=round2(agvs[i].pos_X,3);
  aux_ry:=round2(agvs[i].pos_y,3);
  if aux_step_complete=1 then begin
  if tom[i]=1 then begin
        r:=0;
   end else if tom[i]=2 then begin
  if ((CaminhosAgvs[i].coords[s[i]].node=agvs[i].inicial_node) and  (agvs[i].Direction=CaminhosAgvs[i].coords[s[aux1]].direction)) then
   begin
        r:=0;
   end else begin
       r:=1;
   end;
  end else if tom[i]=3 then begin
      if (((aux_rx>=aux_x-t_d) and(aux_rx<=aux_x+t_d)) and ((aux_ry>=aux_y-t_d) and (aux_ry<=aux_y+t_d)) and (agvs[i].Direction=CaminhosAgvs[i].coords[s[aux1]].direction) ) then
   begin
        r:=0;
   end else begin
       r:=1;
   end;
  end;
  end;
  end;
  end else begin
  r:=0;
  end;
  check_if_still_on_path:=0;
end;
procedure ChangeRobotPriorities(var Map:TEAstar.TAStarMap;var agvs:r_node);
var
    aux_agv: TEAstar.Robot_Pos_info;
    p_id,aux1:integer;
begin
        //Changes the robots priorities to prevent situations where one of the robots would be incapable of executing its mission because it gets block
        //By the robots with higher priorities
        noPath:=false;
        trocas:=trocas+1;
        p_id:=agvs[robotNoPlan].InitialIdPriority;
        if p_id>1 then
        begin
        aux1:=return_id_frompriority(agvs,p_id-1);
        agvs[robotNoPlan].InitialIdPriority:=p_id-1;
        agvs[aux1].InitialIdPriority:=p_id;
        robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS,CaminhosAgvs_af);
        end;
end;

procedure InverseValidationOfPriorities(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
var
    stepPath: integer;
    v,robo: integer;
    i,j,k,count,steps: integer;
    aux_agv: TEAstar.Robot_Pos_info;
    p_id,aux1:integer;
begin

     //inverse validation of the planning to avoid that a robot with lower priority
     //can reach the target early and intersect the path of a robot with higher
     //priority; the idea is detect this situations and change the priority again;
     //in this situation, after exchange priorities, plans again and allow to put
     //the target as obstacle in every layer to avoid undefined exchanges

     trocas:=0;
     robo:=NUMBER_ROBOTS-1;
     while robo > 0 do begin
       v:=robo-1;
       while v >= 0 do begin
         stepPath:=CaminhosAgvs[v].steps;
         while stepPath > 0 do begin
           i:=CaminhosAgvs[v].coords[stepPath].node;
           k:=CaminhosAgvs[v].coords[stepPath].steps;
           if ((blocked_node(i,agvs[robo].target_node)=1) and (k>CaminhosAgvs[robo].steps))
               or ((i=agvs[robo].target_node) and (k>CaminhosAgvs[robo].steps))
           then begin

              robotNoPlan:=robo;
              while ((robotNoPlan > 0) and (trocas < MAX_EXCHANGES)) do begin

                noPath:=false;
                trocas:=trocas+1;
                p_id:=agvs[robotNoPlan].InitialIdPriority;
                if p_id>1 then
                begin
                aux1:=return_id_frompriority(agvs,p_id-1);
                agvs[robotNoPlan].InitialIdPriority:=p_id-1;
                agvs[aux1].InitialIdPriority:=p_id;
                robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS,CaminhosAgvs_af);
                end;

                robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS,CaminhosAgvs_af);

              end;

           end;
           stepPath:=stepPath-1;
         end;
         v:=v-1;
       end;
       robo:=robo-1;
     end;

end;

//function checkcaminhos(var CaminhosAgvs:Caminhos):integer;
//
//var
// n1,aux1,aux2,r:integer;
//begin
//  if robotNoPlan=0 then begin
//  r:=0;
//  for aux1:=0 to NUMBER_ROBOTS-1 do begin
//     n1:=CaminhosAgvs[aux1].coords[2].node;
//     for aux2:=0 to NUMBER_ROBOTS-1 do begin
//        n2:=CaminhosAgvs[aux1].coords[1].node;
//        if n1=n2 then begin
//
//        end;
//     end;
//  end;
// end;
//end;


procedure TEArun(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
var
    aux1:integer;
begin
  //Main function that executes the TEA* path planning

  flagTargetOverlapInverse:=false;

    robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS,CaminhosAgvs_af);

    //robotNoPlan:= checkcaminhos(CaminhosAgvs);
    if ((robotNoPlan <> -1)) then begin
        trocas:=0;
        while ((robotNoPlan > 0) and (trocas < MAX_EXCHANGES)) do begin
              ChangeRobotPriorities(Map,agvs);
              flagChange:=true;
        end;
        totalTrocas:=trocas;
    end;

    trocas:=0;
    flagTargetOverlapInverse := true;
    InverseValidationOfPriorities(Map,agvs,CaminhosAgvs);
    if trocas > 0 then begin
        flagChange:=true;
    end;
    totalValidations:=trocas;
    //removenodes(map,c_cnodes);
    c_cnodes:=0;
    for aux1:=0 to NUMBER_ROBOTS-1 do begin
        current_step[aux1]:=1;
    end;
    type_of_movement:=get_t_mov(CaminhosAgvs,1);
end;

//function DistToReference(robot:integer;xCam:double;yCam:double):double;
//begin
//   if followLine[robot] = true then begin
//       case directionDest[robot] of
//       0: begin
//               result:=xCam-(xDest[robot])*CELLSCALE;
//          end;
//       2: begin
//               result:=(yDest[robot])*CELLSCALE-yCam;
//          end;
//       4: begin
//               result:=(xDest[robot])*CELLSCALE-xCam;
//          end;
//       6: begin
//               result:=yCam-(yDest[robot])*CELLSCALE;
//          end;
//       end;
//   end
//   else if followCircle[robot] = true then begin
//        result := sqrt(sqr(xCam-rotationCenter[robot].x) + sqr(yCam-rotationCenter[robot].y))-CELLSCALE;
//   end;
//
//end;

//function AngleToReference(robot:integer;xCam:double;yCam:double;thetaCam:double):double;
//var
//    r_x,r_y: double;
//    alpha: double;
//    produtointerno_rs,norma_r,norma_s: double;
//begin
//    if followLine[robot] = true then begin
//        result:=DiffAngle(thetaDest[robot],thetaCam);
//    end
//    else if followCircle[robot] = true then begin
//        r_x:=xCam-rotationCenter[robot].x;
//        r_y:=yCam-rotationCenter[robot].y;
//
//        //ângulo entre o vetor r e o vetor s (calculado antes do movimento circular)
//        produtointerno_rs := r_x*s[robot].x + r_y*s[robot].y;
//        norma_r := sqrt(sqr(r_x) + sqr(r_y));
//        norma_s := sqrt(sqr(s[robot].x) + sqr(s[robot].y));
//        alpha := arccos(produtointerno_rs / (norma_r*norma_s));
//
//        result := DiffAngle(DiffAngle(thetaCam,thetaDest[robot]),(pi/2-alpha));
//        if (result > (pi-THRESHOLD_ANGLE)) then begin
//            result:=result-pi;
//        end
//        else if (result < (-pi+THRESHOLD_ANGLE)) then begin
//            result:=result+pi;
//        end;
//    end;
//
//end;

//function Signal(value:integer):integer;
//begin
//     if value > 0 then begin
//         result:=1;
//     end
//     else if value < 0 then begin
//         result:=-1;
//     end;
//end;

//procedure UpdateThetaDest(robot:integer;thetaCam:double);
//begin
//   if ((thetaDest[robot]-thetaCam < 0))
//   then begin
//       thetaDest[robot]:=thetaDest[robot]-pi*0.25;
//   end
//   else if ((thetaDest[robot]-thetaCam > 0))
//   then begin
//       thetaDest[robot]:=thetaDest[robot]+pi*0.25;
//   end;
//end;
//
//procedure UpdateThetaDestPi(robot:integer;thetaCam:double);
//begin
//    if thetaDest[robot] >= 0 then begin
//        thetaDest[robot]:=thetaDest[robot] - pi;
//    end
//    else if thetaDest[robot] < 0 then begin
//        thetaDest[robot]:=thetaDest[robot] + pi;
//    end;
//end;
//
//procedure UpdateThetaDestToMoveBack(robot:integer;thetaCam:double);
//begin
//     if ((thetaDest[robot]-thetaCam) > (pi + THRESHOLD_ANGLE))
//     then begin
//         thetaDest[robot]:=thetaDest[robot]-pi*2;
//     end
//     else if ((thetaDest[robot]-thetaCam) < (-pi - THRESHOLD_ANGLE))
//     then begin
//         thetaDest[robot]:=thetaDest[robot]+pi*2;
//     end;
//
//     if ((thetaDest[robot]-thetaCam < 0))
//     then begin
//         thetaDest[robot]:=thetaDest[robot]+pi*0.25;
//     end
//     else if ((thetaDest[robot]-thetaCam > 0))
//     then begin
//         thetaDest[robot]:=thetaDest[robot]-pi*0.25;
//     end;
//     UpdateThetaDestPi(robot,thetaCam);
//end;
//
//procedure UpdateThetaDestAfterPiRotation(robot:integer;thetaCam:double);
//begin
//   if ((thetaDest[robot]-thetaCam < 0))
//   then begin
//       thetaDest[robot]:=thetaDest[robot]+pi*0.25-pi;
//   end
//   else if ((thetaDest[robot]-thetaCam > 0))
//   then begin
//       thetaDest[robot]:=thetaDest[robot]-pi*0.25+pi;
//   end;
//end;

function checkinterval(x:double;y:double):integer;
var
    r,s1,aux1:integer;
begin
   //Checks if the value is withing the map boundries
   s1:=length(form2.X_c_max);
   r:=0;
   for aux1:=0 to s1-1 do begin
     if ((form2.coms_flaws=1) and (form2.X_c_min[aux1]<=x) and (form2.X_c_max[aux1]>=x) and  (form2.Y_c_min[aux1]<=y) and (form2.Y_c_max[aux1]>=y)) then begin
     r:=1;
     break;
     end;
   end;
   checkinterval:=r;
end;


//SIMTWO
procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer;data:string);
var
    i,j:integer;
    X_a,Y_a:Double;
    c_a:integer;
    xCamStr,id_robstr,coms_countstr,yCamStr,thetaCamStr:string;
begin
   //Unpacks and interpects the data sent by the simtwo simulation software
    j:=2;
    if (data<>'') then begin
      if (data[j]<>'F') then begin
           while (data[j]<>'C') do begin
               id_robstr := id_robstr + data[j];
               j:=j+1;
            end;
            j:=j+1;
            while (data[j]<>'X') do begin
               coms_countstr := coms_countstr + data[j];
               j:=j+1;
            end;
            j:=j+1;
            while (data[j]<>'Y') do begin
               xCamStr := xCamStr + data[j];
               j:=j+1;
            end;

            j:=j+1;
            while (data[j]<>'T') do begin
               yCamStr := yCamStr + data[j];
               j:=j+1;
            end;

            j:=j+1;
            while ((data[j]<>'F')) do begin
               thetaCamStr := thetaCamStr + data[j];
               j:=j+1;
            end;

            X_a:=StrToFloat(xCamStr);
            Y_a:=StrToFloat(yCamStr);
            if ((checkinterval(X_a,Y_a)=1) or ((form2.coms_flaws_random=1) and (random(99)<f_percent))) then
            begin
            id_rob:= Strtoint(id_robstr);
            a_c_flaw[id_rob-1]:=1;
            xCam[id_rob-1] := 9999999;
            yCam[id_rob-1] := 9999999;
            thetaCam[id_rob-1] := 9999999;
            end else begin
            id_rob:= Strtoint(id_robstr);
            c_a:=Strtoint(coms_countstr);
            xCam[id_rob-1] := StrToFloat(xCamStr);
            yCam[id_rob-1] := StrToFloat(yCamStr);
            thetaCam[id_rob-1] := StrToFloat(thetaCamStr);
            a_c_flaw[id_rob-1]:=0;
            if c_a=999 then begin
              coms_count[id_rob-1]:=0;
            end else begin
               coms_count[id_rob-1]:=c_a;
            end;
            end;
            xCamStr:='';
            yCamStr:='';
            thetaCamStr:='';
            id_robstr:='';
            coms_countstr:='';
         end;
      end;
end;



procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer);
var
    s1,i:integer;
begin
        //Updates the robot position and associates it to a node of the graph that represents the factory floor map
        i:=id_rob-1;
        form1.robots[i].pos_X := xCam[i];
        form1.robots[i].pos_Y:= yCam[i];
        if init[i]=0 then
        begin
        form1.robots[i].inicial_node:=get_closest_node_id(form1.full_nodelist,form1.robots[i].pos_X ,form1.robots[i].pos_Y,1);
        if check_array( unflawed_node,form1.robots[i].inicial_node)=0 then begin
        s1:=length( unflawed_node);
        setlength( unflawed_node,s1+1);
        unflawed_node[s1]:=form1.robots[i].inicial_node;
        flaw_location:=check_if_flaw_ceased(form1.robots[i].inicial_node,flaw_location);
        end;
            init[i]:=checkfornode(form1.full_nodelist, form1.robots[i].pos_X ,form1.robots[i].pos_Y,1);
         //init[i]:=0;
        end
        else
        begin
        form1.robots[i].inicial_node:=updaterobotnode(form1.full_nodelist,form1.robots[i].pos_X ,form1.robots[i].pos_Y,1,i);
        end;
        if ((thetaCam[i] <= pi/2 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi/2 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 0;
        end
        else if ((thetaCam[i] <= pi/4 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi/4 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 1;
        end
        else if ((thetaCam[i] <= 0 + THRESHOLD_ANGLE) and (thetaCam[i] >= 0 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 2;
        end
        else if ((thetaCam[i] <= -pi/4 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi/4 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 3;
        end
        else if ((thetaCam[i] <= -pi/2 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi/2 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 4;
        end
        else if ((thetaCam[i] <= -pi*1.25 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi*1.25 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction := 5;
        end
        else if (((thetaCam[i] <= pi + THRESHOLD_ANGLE) and (thetaCam[i] >= pi - THRESHOLD_ANGLE)) or
                 ((thetaCam[i] <= -pi + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi - THRESHOLD_ANGLE)))
        then begin
              form1.robots[i].Direction := 6;
        end
        else if ((thetaCam[i] <= pi*1.25 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi*1.25 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 7;
        end;
end;



procedure UpdateSubmissions(var agvs:R_NODE;i:integer);
begin
    //Updates the current mission of the robot
    if ((agvs[i].inicial_node <> agvs[i].SubMissions[agvs[i].NumberSubMissions-1]))
    then begin

        if ((agvs[i].inicial_node = agvs[i].SubMissions[agvs[i].ActualSubMission-1]) and
            (agvs[i].ActualSubMission < agvs[i].NumberSubMissions))
        then begin
            agvs[i].ActualSubMission := agvs[i].ActualSubMission + 1;
            agvs[i].CounterSubMissions := agvs[i].CounterSubMissions + 1;
        end;
         agvs[i].onrest:=0;
         f_rest[i]:=0;
    end else begin
     agvs[i].onrest:=1;
     if f_rest[i]=0 then begin
     f_replan:=1;
     f_rest[i]:=1;
     end;
    end;
end;

procedure Update_missions_in_flaw(var agvs:R_NODE;CaminhosAgvs_flaw:Caminhos;r:integer;robots_flaws:cf_robots;s:integer);
var
aux_id,a1,id1,se1,a2,id2,se2,s1:integer;
begin
   //Updates the current mission of the robot, when the robot concludes its task inside an area where no communication exists
   s1:=length(CaminhosAgvs_flaw[r].coords);
   a1:=9999999;
   if ((robots_flaws[r].curr_in_node=robots_flaws[r].curr_out_node) and (robots_flaws[r].active_consecutive_hits>1)) then begin
   for aux1:=s+1 to s1-1 do begin
         if CaminhosAgvs_flaw[r].coords[aux1].node=robots_flaws[r].curr_in_node then begin
              a2:=aux1;
              id1:=CaminhosAgvs_flaw[r].coords[aux1].node;
              se1:=CaminhosAgvs_flaw[r].coords[aux1].steps;
              break;
         end;
   end;
   end else begin
   for aux1:=s to s1-1 do begin
         if CaminhosAgvs_flaw[r].coords[aux1].node=robots_flaws[r].curr_out_node then begin
              a2:=aux1;
              id1:=CaminhosAgvs_flaw[r].coords[aux1].node;
              se1:=CaminhosAgvs_flaw[r].coords[aux1].steps;
              break;
         end;
   end;
end;

   for aux1:=s to a2 do begin
      aux_id:=CaminhosAgvs_flaw[r].coords[aux1].node;
      if ((aux_id<> agvs[r].SubMissions[agvs[r].NumberSubMissions-1]))
         then begin
        if ((aux_id = agvs[r].SubMissions[agvs[r].ActualSubMission-1]) and
            (agvs[r].ActualSubMission < agvs[r].NumberSubMissions))
        then begin
            agvs[r].ActualSubMission := agvs[r].ActualSubMission + 1;
            agvs[r].CounterSubMissions := agvs[r].CounterSubMissions + 1;
        end;
    end;
   end;
end;

function max_diff_step(var a_i:m_node):integer;
var
min,max,s1,diff,aux1:integer;
begin
   //Checks if the all robots are within the maximoun accepted step distance from each other
    s1:=length(a_i);
    min:=9999999;
    max:=0;
    for aux1:=0 to s1-1 do begin
       if ((a_i[aux1]>=max) and (form1.robots[aux1].onrest<>1)) then begin
          max:=a_i[aux1];
       end;
        if ((a_i[aux1]<=min) and (form1.robots[aux1].onrest<>1)) then begin
          min:=a_i[aux1];
       end;
    end;
    diff:=max-min;
    max_diff_step:=diff
end;

function checkforflaws(rf:cf_robots):integer;
var
aux1,r,s1:integer;
begin
 //Checks if the current robot is in a fault situation
 s1:=length(rf);
 r:=0;
 for aux1:=0 to s1-1 do begin
     if rf[aux1].isactive=1 then begin
     r:=1;
     end;
 end;
 checkforflaws:=r;
end;



{ TFControlo }

procedure TFControlo.udpComReceive(aSocket: TLSocket);
var
    data : string;
    i : integer;
    ttttt:integer;
    xCam,yCam,thetaCam: array[0..NUMBER_ROBOTS-1] of double;
    s1,aux_exit_flaw_node,cont,cont2,aux_step_complete,s4,n_curr,aux1,idf:integer;

begin
    //Communication reception of messages
    t1:=GetTickCount64();
    udpCom.GetMessage(data);

    if data <> '' then begin
      Edit1.Text:= data;
      Edit1.Color:= clgreen;
    end else begin
      Edit1.Text:= 'Erro';
      Edit1.Color:= clred;
    end;

    if data <> '' then begin
       Edit2.Text:=data[1];
    end;


    Edit8.Text:=FloatToStr(linearVelocities[0]);
    Edit9.Text:=FloatToStr(linearVelocities[1]);


    if (data <> '') then begin
        if ((flagVelocities = true) and (data[1] = 'R')) then begin
          Edit2.Text:='Hello';
          //Analyse of the received messages
          UnpackUDPmessage(xCam,yCam,thetaCam,id_rob,data);

          if ((xCam[id_rob-1]<>9999999) and (yCam[id_rob-1]<>9999999) and (thetaCam[id_rob-1]<>9999999)) then begin
          //update of the robot position and of it's current submissions
          UpdateInitialPoints(xCam,yCam,thetaCam,id_rob);
         // ttttt:=unflawed_node[0];
          UpdateSubmissions(form1.robots,id_rob-1);
          timestamp_coms[id_rob-1]:=total_seconds;
          //Checks if the robot as a  fault active (fault exit control)
          if robots_flaws[id_rob-1].isactive=1 then begin
          s4:=length(form1.full_nodelist);
          idf:=0;
          for aux1:=0 to s4-1 do
           begin
           //Updates the fault dimention
           n_curr:=form1.full_nodelist[aux1].id;
           if n_curr=form1.robots[id_rob-1].inicial_node then
           begin
           idf:=n_curr;
            end;
           end;
           if idf=0 then begin
           //aux_exit_flaw_node:=get_linked_node(form1.full_nodelist,agvs[i].pos_X,agvs[i].pos_y,1,CaminhosAgvs[i].coords[1].node);
           //aux_exit_flaw_node:=get_futhrest_exit_link_node(form1.map,form1.robots,id_rob-1,CaminhosAgvs_af,current_step_f[id_rob-1],unflawed_node);
           end else begin
           aux_exit_flaw_node:=idf;
           robots_flaws[id_rob-1].curr_out_node:=aux_exit_flaw_node;
           Update_missions_in_flaw(form1.robots,CaminhosAgvs_af,id_rob-1,robots_flaws,current_step_f[id_rob-1]);
           f_replan:=1;
           if robots_flaws[id_rob-1].isdetecting=1 then begin
              robots_flaws[id_rob-1].isdetecting:=0;
              flaw_location:=save_exit_node(form1.map,form1.robots,id_rob-1,robots_flaws,flaw_location,unflawed_node);
                //robots_flaws[id_rob-1].detected_nodes:=update_flaw_path(form1.robots,id_rob-1,CaminhosAgvs);
             end else begin
                 if check_array(flaw_location.in_node[robots_flaws[id_rob-1].flaw_ind],aux_exit_flaw_node)=1 then begin
                end else begin
                  flaw_location:=update_afected_nodes(flaw_location,robots_flaws,id_rob-1,unflawed_node,current_step_f[id_rob-1]);
                end;
                end;

                robots_flaws[id_rob-1].isactive:=0;
          end;
          end;
          end;
          i:=0;
          while i<NUMBER_ROBOTS do begin
              //Normal communication with the robot
              if ((type_of_movement[i]=0)) then begin
                 //Initiates the TEA* planning algorithm
                 Edit4.Text:='TEA_Init';
                  followLine[i]:=false;
                  followCircle[i]:=false;
                  rotate[i]:=false;

                  UpdateSubmissions(form1.robots,i);

                  TEArun(form1.map,form1.robots,CaminhosAgvs);
                  for aux1:=0 to NUMBER_ROBOTS-1 do begin
                  current_step[aux1]:=1;
                  end;
                  //Detects a priority change and leaves the cycle so that the information isn't store with the wrong indexes
                  if ((flagChange = true) and (totalTrocas < MAX_EXCHANGES) and (totalTrocas <> totalValidations)) then begin
                    flagChange:=false;
                    totalTrocas:=0;
                    totalValidations:=0;
                    break;
                  end
                  else begin
                    trocas:=0;
                    totalTrocas:=0;
                  end;
                  step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
                  //timestamp_coms[i]:=total_seconds;
              end else if ((timestamp_coms[i]+2<=total_seconds) and (type_of_movement[i]<>0) and (timestamp_coms[i]<>0))  then begin

              //coms flaw no communication with robot
              if robots_flaws[i].isactive=0 then begin
               flaw_location:=save_flaw_location(form1.map,CaminhosAgvs,form1.robots,i,form1.full_nodelist,current_step[i],flaw_location,robots_flaws,unflawed_node);
               //robots_flaws[i].isactive:=1;
               ttttt:=length(flaw_location.in_node[0]);
               current_step_f[i]:= current_step[i];
               f_replan:=1;
               end else begin
                robots_flaws[i].active_consecutive_hits:= robots_flaws[i].active_consecutive_hits+1;
               end;
               timestamp_coms[i]:=total_seconds;
              end else begin
              step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
              if i=0 then begin
              Edit7.Text:=inttostr(current_step[i]);
              end else if i=1 then begin
              Edit8.Text:=inttostr(current_step[i]);
              end else if i=2 then begin
              Edit8.Text:=inttostr(current_step[i]);
              end else if i=3 then begin
              Edit9.Text:=inttostr(current_step[i]);
              end;
              {if ((step_complete[i]=0) or  (type_of_movement[i]=1)) then begin
                  Edit4.Text:='MOVE';
              end}
               if ((check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=1) or (f_replan=1) or (max_diff_step(current_step)>1) or (check_if_too_ahead_on_step(CaminhosAgvs,Form1.robots,current_step)=1) or (check_if_too_close(form1.robots)=1) {or (checkforflaws(robots_flaws)=1)}) then begin
                 // if check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=1 then
                 // begin
                  b_pathsend:=1;  //blocks the sending of a error filled path mid planning

                  Edit4.Text:='TEA';

                  //UpdateSubmissions(form1.robots,i);

                  TEArun(form1.map,form1.robots,CaminhosAgvs);
                  for aux1:=0 to NUMBER_ROBOTS-1 do begin
                         current_step[aux1]:=1;
                         //current_step_s[aux1]:=1;
                  end;

                  //Detects a priority change and leaves the cycle so that the information isn't store with the wrong indexes
                  if ((flagChange = true) and (totalTrocas < MAX_EXCHANGES) and (totalTrocas <> totalValidations)) then begin
                    flagChange:=false;
                    totalTrocas:=0;
                    totalValidations:=0;
                    break;
                  end
                  else begin
                    trocas:=0;
                    totalTrocas:=0;
                  end;
                  step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
                  f_replan:=0;
                  b_pathsend:=0;
                 { end else if check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=0 then
                  begin
                     Edit4.Text:='Next_Step';
                     //current_step[i]:=current_step[i]+1;
                 end;}
                 break;
              end else begin
                    if ((step_complete[i]=0) and  (type_of_movement[i]<>1)) then begin
                        Edit4.Text:='MOVE';
                     end else if ((type_of_movement[i]=1))then begin
                         cont:=0;
                         cont2:=0;
                         for aux1:=0 to NUMBER_ROBOTS-1 do begin
                          if type_of_movement[aux1]<>1 then begin
                             cont:=cont+1;
                          end;
                          if ((step_complete[aux1]=1) or (current_step[aux1]>current_step[i]) and (aux1<>i)) then begin
                             cont2:=cont2+1;
                          end;
                      end;
                         if ((cont2>=cont) and ((checkforflaws(robots_flaws)=0))) then begin
                           Edit4.Text:='Next_Step';
                           // UpdateSubmissions(form1.robots,i);
                           current_step[i]:=current_step[i]+1;
                           end;

                     end else  begin
                         Edit4.Text:='Next_Step';
                         //UpdateSubmissions(form1.robots,i);
                         current_step[i]:=current_step[i]+1;
                     end;

              end;
              end;
              i:=i+1;
          end;
        end;
    end;

    current_step_s:=current_step;
    CaminhosAgvs_s:=CaminhosAgvs;
    if data = 'MIP1' then begin
        flagMessageInitialPositions:=false;
        //flagVelocities:=true;
        Edit3.Text:='MIP1';
    end;
    t2:=GetTickCount64();
    tick_p:=t2-t1;
    Edit10.Text:=IntToStr(tick_p);
end;
procedure TFControlo.TimerSendTimer(Sender: TObject);
var

    l1:integer;
    aux1:integer;
    count:integer;
    messagelist:Tstringlist;
begin
   //Sending of information regarding the robots paths based on a timer
    if flagMessageInitialPositions = true then begin
       udpCom.SendMessage(MessageInitialPositions, '127.0.0.1:9808');
       ind_robo:=0;
    end;
    if ((form2.coms_flaws_random=1) and (random(99)<f_percent)) then begin
      END ELSE if ((flagVelocities = true) {and (b_pathsend<>1)}) then begin

        if ((ind_robo<NUMBER_ROBOTS) and (robots_flaws[ind_robo].isactive<>1))  then begin
           if a_c_flaw[ind_robo]=0 then begin
           messagelist:=TStringList.Create;
           Ca[ind_robo]:=checkforpathcompletion(form1.robots,ind_robo);
           messagelist.Add('N' + IntToStr( form1.robots[ind_robo].id_robot));
           messagelist.Add('P' + IntToStr( form1.robots[ind_robo].InitialIdPriority));
           messagelist.Add('S' + IntToStr(get_steps(CaminhosAgvs_s,ind_robo)-1));
           l1:=length((CaminhosAgvs_s[ind_robo].coords));
           count:=1;
           for aux1:=current_step_s[ind_robo] to l1-1 do begin
           if ((getXcoord(CaminhosAgvs_s[ind_robo].coords[aux1].node)<3) and (getYcoord(CaminhosAgvs_s[ind_robo].coords[aux1].node)<3) and (count<15) and (CaminhosAgvs_s[ind_robo].coords[aux1].node<>0)) then
           begin
           messagelist.Add('I' + IntToStr(count));
           messagelist.Add('X' + FloatToStr(round2(getXcoord(CaminhosAgvs_s[ind_robo].coords[aux1].node),3)));
           messagelist.Add('Y' + FloatToStr(round2(getYcoord(CaminhosAgvs_s[ind_robo].coords[aux1].node),3)));
           messagelist.Add('D' + IntToStr(CaminhosAgvs_s[ind_robo].coords[aux1].direction));
           count:=count+1;
           end;
           end;
           if count<2 then
           begin
           messagelist.Add('T' + IntToStr(1));
           end else begin
           messagelist.Add('T' + IntToStr( Ca[ind_robo]));
           end;
           messagelist.Add('F');
           MessageVelocities:=messagelist.Text;
           udpCom.SendMessage(MessageVelocities, '127.0.0.1:9808');
           messagelist.free;
           ind_robo:=ind_robo+1;
           end else begin
            ind_robo:=ind_robo+1;
           end;
        end else begin
        ind_robo:=0;
        end;
        Edit5.Text:=inttostr(form1.robots[0].Direction);
        Edit6.Text:=inttostr(form1.robots[3].Direction);
        //Edit7.Text:=MessageVelocities1;
        MessageVelocities1:='';
        MessageVelocities:='';
        ct:=1;
        s_time:=s_time+80;
        if s_time>=1000 then begin
         total_seconds:=total_seconds+1;
         s_time:=0;
    end;
    end;
end;

procedure TFControlo.FormShow(Sender: TObject);
var
    i:integer;
begin
    Edit1.Text:= 'Error';
    Edit1.Color:= clred;

    udpCom.Disconnect;
    if udpCom.Connect('127.0.0.1', 4040) then begin   //CONEXAO COM O SIMTWO?
        Edit1.Text:= 'connection open';
        Edit1.Color:= clyellow;
    end;
    if udpCom.Listen(4040) then begin
        Edit1.Text:= 'open ports';
    end;

    flagMessageInitialPositions:=true;
    flagVelocities:=false;
    MessageVelocities:='';
    contador:=0;
    trocas:=0;
    flagChange := false;
    totalTrocas := 0;
    totalValidations := 0;
    ind_robo:=0;
    b_pathsend:=0;
    f_replan:=0;
    InitialPointsForAllRobots(form1.robots);


    i:=0;
    while i<NUMBER_ROBOTS do begin
      linearVelocities[i]:=0;
      angularVelocities[i]:=0;
      i:=i+1;
    end;
   ct:=0;
end;

procedure TFControlo.SendButtonClick(Sender: TObject);
var
    i:integer;
begin
    flagVelocities:=true;
    s_time:=0;
    total_seconds:=0;
    for i:=0 to NUMBER_ROBOTS-1 do
    begin
    timestamp_coms[i]:=0;
    type_of_movement[i]:=0;
    current_step[i]:=1;
    ghost_nodes[i]:=0;
    robots_flaws[i].isactive:=0;
    robots_flaws[i].isdetecting:=0;
    robots_flaws[i].active_consecutive_hits:=0;
    end;

end;

procedure TFControlo.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
    udpCom.Disconnect();
end;

procedure TFControlo.FormCreate(Sender: TObject);
var
    aux1:integer;
begin
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
  pre_coms_count[aux1]:=10000
  end;
end;

end.
