// Sparse, frame-stable road completion. No image DDR requests or texture RAM.
// 23 controls per side, RGB565 code values with four fractional bits.
// Collection uses already-read REAL pixels. Postprocessing runs only between
// render frames; the renderer must wait for busy=0 before starting a frame.
module bev_road_surface(
    input wire clk, rst_n,
    input wire frame_begin, frame_end,
    input wire [3:0] source_ready,
    input wire sample_valid,
    input wire [9:0] sample_x, sample_y,
    input wire [15:0] sample_pixel,
    output wire edge_sample_valid,
    input wire query_start,
    input wire [9:0] query_x, query_y,
    output wire busy,
    output reg fill_valid,
    output reg [15:0] fill_pixel
);
localparam N=23, TOTAL=46;
localparam IDLE=0, N_CHECK=1, N_DIV=2, N_WAIT=3, FIND_INIT=4,
           FIND_SCAN=5, FIND_PICK=6, WEIGHT_WAIT=7, FILL_SLOT=8,
           SMOOTH_READ=9, SMOOTH_WRITE=10, PUBLISH=11;
reg [3:0] state;
assign busy = (state != IDLE);

reg [13:0] sum_r[0:TOTAL-1], sum_g[0:TOTAL-1], sum_b[0:TOTAL-1];
reg [8:0] count[0:TOTAL-1];
reg [29:0] mean_q[0:TOTAL-1], active_q[0:TOTAL-1];
reg [TOTAL-1:0] observed;
reg [1:0] side_has_data;
reg active_valid, collecting, epoch_bad;
reg [3:0] epoch_ready, active_ready;

// Geometry ONLY: packed left/right boundaries relative to road x=90.
// 9'h1ff means no real road on that side of this row. Generated from the
// current bev_coord_map coefficients; regenerate after changing the mapper.
reg [17:0] boundary_rom[0:719];
// BEGIN GENERATED ROAD BOUNDS
initial begin
    boundary_rom[0]=18'h0fdff;
    boundary_rom[1]=18'h0fdff;
    boundary_rom[2]=18'h0fdff;
    boundary_rom[3]=18'h0ffff;
    boundary_rom[4]=18'h0ffff;
    boundary_rom[5]=18'h0ffff;
    boundary_rom[6]=18'h101ff;
    boundary_rom[7]=18'h101ff;
    boundary_rom[8]=18'h103ff;
    boundary_rom[9]=18'h103ff;
    boundary_rom[10]=18'h103ff;
    boundary_rom[11]=18'h105ff;
    boundary_rom[12]=18'h105ff;
    boundary_rom[13]=18'h105ff;
    boundary_rom[14]=18'h107ff;
    boundary_rom[15]=18'h107ff;
    boundary_rom[16]=18'h107ff;
    boundary_rom[17]=18'h109ff;
    boundary_rom[18]=18'h109ff;
    boundary_rom[19]=18'h10bff;
    boundary_rom[20]=18'h10bff;
    boundary_rom[21]=18'h10bff;
    boundary_rom[22]=18'h10dff;
    boundary_rom[23]=18'h10dff;
    boundary_rom[24]=18'h10dff;
    boundary_rom[25]=18'h10fff;
    boundary_rom[26]=18'h10fff;
    boundary_rom[27]=18'h111ff;
    boundary_rom[28]=18'h111ff;
    boundary_rom[29]=18'h111ff;
    boundary_rom[30]=18'h113ff;
    boundary_rom[31]=18'h113ff;
    boundary_rom[32]=18'h113ff;
    boundary_rom[33]=18'h115ff;
    boundary_rom[34]=18'h115ff;
    boundary_rom[35]=18'h115ff;
    boundary_rom[36]=18'h117ff;
    boundary_rom[37]=18'h117ff;
    boundary_rom[38]=18'h119ff;
    boundary_rom[39]=18'h119ff;
    boundary_rom[40]=18'h119ff;
    boundary_rom[41]=18'h11bff;
    boundary_rom[42]=18'h11bff;
    boundary_rom[43]=18'h11bff;
    boundary_rom[44]=18'h11dff;
    boundary_rom[45]=18'h11dff;
    boundary_rom[46]=18'h11dff;
    boundary_rom[47]=18'h11fff;
    boundary_rom[48]=18'h11fff;
    boundary_rom[49]=18'h11dff;
    boundary_rom[50]=18'h11dff;
    boundary_rom[51]=18'h11bff;
    boundary_rom[52]=18'h119ff;
    boundary_rom[53]=18'h119ff;
    boundary_rom[54]=18'h117ff;
    boundary_rom[55]=18'h115ff;
    boundary_rom[56]=18'h115ff;
    boundary_rom[57]=18'h113ff;
    boundary_rom[58]=18'h111ff;
    boundary_rom[59]=18'h10fff;
    boundary_rom[60]=18'h10fff;
    boundary_rom[61]=18'h10dff;
    boundary_rom[62]=18'h10bff;
    boundary_rom[63]=18'h10bff;
    boundary_rom[64]=18'h109ff;
    boundary_rom[65]=18'h107ff;
    boundary_rom[66]=18'h107ff;
    boundary_rom[67]=18'h105ff;
    boundary_rom[68]=18'h103ff;
    boundary_rom[69]=18'h101ff;
    boundary_rom[70]=18'h101ff;
    boundary_rom[71]=18'h0ffff;
    boundary_rom[72]=18'h0fdff;
    boundary_rom[73]=18'h0fdff;
    boundary_rom[74]=18'h0fbff;
    boundary_rom[75]=18'h0f9ff;
    boundary_rom[76]=18'h0f9ff;
    boundary_rom[77]=18'h0f7ff;
    boundary_rom[78]=18'h0f5ff;
    boundary_rom[79]=18'h0f3ff;
    boundary_rom[80]=18'h0f3ff;
    boundary_rom[81]=18'h0f1ff;
    boundary_rom[82]=18'h0efff;
    boundary_rom[83]=18'h0efff;
    boundary_rom[84]=18'h0edff;
    boundary_rom[85]=18'h0ebff;
    boundary_rom[86]=18'h0ebff;
    boundary_rom[87]=18'h0e9ff;
    boundary_rom[88]=18'h0e7ff;
    boundary_rom[89]=18'h0e5ff;
    boundary_rom[90]=18'h0e5ff;
    boundary_rom[91]=18'h0e3ff;
    boundary_rom[92]=18'h0e1ff;
    boundary_rom[93]=18'h0e1ff;
    boundary_rom[94]=18'h0dfff;
    boundary_rom[95]=18'h0ddff;
    boundary_rom[96]=18'h0ddff;
    boundary_rom[97]=18'h0dbff;
    boundary_rom[98]=18'h0d9ff;
    boundary_rom[99]=18'h0d7ff;
    boundary_rom[100]=18'h0d7ff;
    boundary_rom[101]=18'h0d5ff;
    boundary_rom[102]=18'h0d3ff;
    boundary_rom[103]=18'h0d3ff;
    boundary_rom[104]=18'h0d1ff;
    boundary_rom[105]=18'h0cfff;
    boundary_rom[106]=18'h0cfff;
    boundary_rom[107]=18'h0cdff;
    boundary_rom[108]=18'h0cbff;
    boundary_rom[109]=18'h0c9ff;
    boundary_rom[110]=18'h0c9ff;
    boundary_rom[111]=18'h0c7ff;
    boundary_rom[112]=18'h0c5ff;
    boundary_rom[113]=18'h0c5ff;
    boundary_rom[114]=18'h0c3ff;
    boundary_rom[115]=18'h0c1ff;
    boundary_rom[116]=18'h0c1ff;
    boundary_rom[117]=18'h0bfff;
    boundary_rom[118]=18'h0bdff;
    boundary_rom[119]=18'h0bbff;
    boundary_rom[120]=18'h0bbff;
    boundary_rom[121]=18'h0b9ff;
    boundary_rom[122]=18'h0b7ff;
    boundary_rom[123]=18'h0b7ff;
    boundary_rom[124]=18'h0b5ff;
    boundary_rom[125]=18'h0b3ff;
    boundary_rom[126]=18'h0b1ff;
    boundary_rom[127]=18'h0b1ff;
    boundary_rom[128]=18'h0afff;
    boundary_rom[129]=18'h0adff;
    boundary_rom[130]=18'h0adff;
    boundary_rom[131]=18'h0abff;
    boundary_rom[132]=18'h0a9ff;
    boundary_rom[133]=18'h0a9ff;
    boundary_rom[134]=18'h0a7ff;
    boundary_rom[135]=18'h0a5ff;
    boundary_rom[136]=18'h0a3ff;
    boundary_rom[137]=18'h0a3ff;
    boundary_rom[138]=18'h0a1ff;
    boundary_rom[139]=18'h09fff;
    boundary_rom[140]=18'h09fff;
    boundary_rom[141]=18'h09dff;
    boundary_rom[142]=18'h09bff;
    boundary_rom[143]=18'h09bff;
    boundary_rom[144]=18'h099ff;
    boundary_rom[145]=18'h097ff;
    boundary_rom[146]=18'h095ff;
    boundary_rom[147]=18'h095ff;
    boundary_rom[148]=18'h093ff;
    boundary_rom[149]=18'h091ff;
    boundary_rom[150]=18'h091ff;
    boundary_rom[151]=18'h08fff;
    boundary_rom[152]=18'h08dff;
    boundary_rom[153]=18'h08dff;
    boundary_rom[154]=18'h08bff;
    boundary_rom[155]=18'h089ff;
    boundary_rom[156]=18'h087ff;
    boundary_rom[157]=18'h087ff;
    boundary_rom[158]=18'h085ff;
    boundary_rom[159]=18'h083ff;
    boundary_rom[160]=18'h083ff;
    boundary_rom[161]=18'h081ff;
    boundary_rom[162]=18'h07fff;
    boundary_rom[163]=18'h07fff;
    boundary_rom[164]=18'h07dff;
    boundary_rom[165]=18'h07bff;
    boundary_rom[166]=18'h079ff;
    boundary_rom[167]=18'h079ff;
    boundary_rom[168]=18'h077ff;
    boundary_rom[169]=18'h075ff;
    boundary_rom[170]=18'h075ff;
    boundary_rom[171]=18'h073ff;
    boundary_rom[172]=18'h071ff;
    boundary_rom[173]=18'h071ff;
    boundary_rom[174]=18'h06fff;
    boundary_rom[175]=18'h06dff;
    boundary_rom[176]=18'h06bff;
    boundary_rom[177]=18'h06bff;
    boundary_rom[178]=18'h069ff;
    boundary_rom[179]=18'h067ff;
    boundary_rom[180]=18'h067ff;
    boundary_rom[181]=18'h065ff;
    boundary_rom[182]=18'h063ff;
    boundary_rom[183]=18'h063ff;
    boundary_rom[184]=18'h061ff;
    boundary_rom[185]=18'h05fff;
    boundary_rom[186]=18'h05dff;
    boundary_rom[187]=18'h05dff;
    boundary_rom[188]=18'h05bff;
    boundary_rom[189]=18'h059ff;
    boundary_rom[190]=18'h059ff;
    boundary_rom[191]=18'h057ff;
    boundary_rom[192]=18'h055ff;
    boundary_rom[193]=18'h055ff;
    boundary_rom[194]=18'h053ff;
    boundary_rom[195]=18'h051ff;
    boundary_rom[196]=18'h04fff;
    boundary_rom[197]=18'h04fff;
    boundary_rom[198]=18'h04dff;
    boundary_rom[199]=18'h04bff;
    boundary_rom[200]=18'h04bff;
    boundary_rom[201]=18'h049ff;
    boundary_rom[202]=18'h047ff;
    boundary_rom[203]=18'h047ff;
    boundary_rom[204]=18'h045ff;
    boundary_rom[205]=18'h043ff;
    boundary_rom[206]=18'h041ff;
    boundary_rom[207]=18'h041ff;
    boundary_rom[208]=18'h03fff;
    boundary_rom[209]=18'h03dff;
    boundary_rom[210]=18'h03dff;
    boundary_rom[211]=18'h03bff;
    boundary_rom[212]=18'h039ff;
    boundary_rom[213]=18'h039ff;
    boundary_rom[214]=18'h037ff;
    boundary_rom[215]=18'h035ff;
    boundary_rom[216]=18'h033ff;
    boundary_rom[217]=18'h033ff;
    boundary_rom[218]=18'h031ff;
    boundary_rom[219]=18'h02fff;
    boundary_rom[220]=18'h02fff;
    boundary_rom[221]=18'h02dff;
    boundary_rom[222]=18'h02bff;
    boundary_rom[223]=18'h02bff;
    boundary_rom[224]=18'h029ff;
    boundary_rom[225]=18'h027ff;
    boundary_rom[226]=18'h025ff;
    boundary_rom[227]=18'h025ff;
    boundary_rom[228]=18'h023ff;
    boundary_rom[229]=18'h021ff;
    boundary_rom[230]=18'h021ff;
    boundary_rom[231]=18'h01fff;
    boundary_rom[232]=18'h01dff;
    boundary_rom[233]=18'h01dff;
    boundary_rom[234]=18'h01bff;
    boundary_rom[235]=18'h019ff;
    boundary_rom[236]=18'h017ff;
    boundary_rom[237]=18'h017ff;
    boundary_rom[238]=18'h015ff;
    boundary_rom[239]=18'h013ff;
    boundary_rom[240]=18'h013ff;
    boundary_rom[241]=18'h011ff;
    boundary_rom[242]=18'h00fff;
    boundary_rom[243]=18'h00fff;
    boundary_rom[244]=18'h00dff;
    boundary_rom[245]=18'h00bff;
    boundary_rom[246]=18'h009ff;
    boundary_rom[247]=18'h009ff;
    boundary_rom[248]=18'h007ff;
    boundary_rom[249]=18'h005ff;
    boundary_rom[250]=18'h005ff;
    boundary_rom[251]=18'h003ff;
    boundary_rom[252]=18'h001ff;
    boundary_rom[253]=18'h001ff;
    boundary_rom[254]=18'h3ffff;
    boundary_rom[255]=18'h3ffff;
    boundary_rom[256]=18'h3ffff;
    boundary_rom[257]=18'h3ffff;
    boundary_rom[258]=18'h3ffff;
    boundary_rom[259]=18'h3ffff;
    boundary_rom[260]=18'h3ffff;
    boundary_rom[261]=18'h3ffff;
    boundary_rom[262]=18'h3ffff;
    boundary_rom[263]=18'h3ffff;
    boundary_rom[264]=18'h3ffff;
    boundary_rom[265]=18'h3ffff;
    boundary_rom[266]=18'h3ffff;
    boundary_rom[267]=18'h3ffff;
    boundary_rom[268]=18'h3ffff;
    boundary_rom[269]=18'h3ffff;
    boundary_rom[270]=18'h3ffff;
    boundary_rom[271]=18'h3ffff;
    boundary_rom[272]=18'h3ffff;
    boundary_rom[273]=18'h3ffff;
    boundary_rom[274]=18'h3ffff;
    boundary_rom[275]=18'h3ffff;
    boundary_rom[276]=18'h3ffff;
    boundary_rom[277]=18'h3ffff;
    boundary_rom[278]=18'h3ffff;
    boundary_rom[279]=18'h3ffff;
    boundary_rom[280]=18'h3ffff;
    boundary_rom[281]=18'h3ffff;
    boundary_rom[282]=18'h3ffff;
    boundary_rom[283]=18'h3ffff;
    boundary_rom[284]=18'h3ffff;
    boundary_rom[285]=18'h3ffff;
    boundary_rom[286]=18'h3ffff;
    boundary_rom[287]=18'h3ffff;
    boundary_rom[288]=18'h3ffff;
    boundary_rom[289]=18'h3ffff;
    boundary_rom[290]=18'h3ffff;
    boundary_rom[291]=18'h3ffff;
    boundary_rom[292]=18'h3ffff;
    boundary_rom[293]=18'h3ffff;
    boundary_rom[294]=18'h3ffff;
    boundary_rom[295]=18'h3ffff;
    boundary_rom[296]=18'h3ffff;
    boundary_rom[297]=18'h3ffff;
    boundary_rom[298]=18'h3ffff;
    boundary_rom[299]=18'h3ffff;
    boundary_rom[300]=18'h3ffff;
    boundary_rom[301]=18'h3ffff;
    boundary_rom[302]=18'h3ffff;
    boundary_rom[303]=18'h3ffff;
    boundary_rom[304]=18'h3ffff;
    boundary_rom[305]=18'h3ffff;
    boundary_rom[306]=18'h3ffff;
    boundary_rom[307]=18'h3ffff;
    boundary_rom[308]=18'h3ffff;
    boundary_rom[309]=18'h3ffff;
    boundary_rom[310]=18'h3ffff;
    boundary_rom[311]=18'h3ffff;
    boundary_rom[312]=18'h3ffff;
    boundary_rom[313]=18'h3ffff;
    boundary_rom[314]=18'h3ffff;
    boundary_rom[315]=18'h3ffff;
    boundary_rom[316]=18'h3ffff;
    boundary_rom[317]=18'h3ffff;
    boundary_rom[318]=18'h3ffff;
    boundary_rom[319]=18'h3ffff;
    boundary_rom[320]=18'h3ffff;
    boundary_rom[321]=18'h3ffff;
    boundary_rom[322]=18'h3ffff;
    boundary_rom[323]=18'h3ffff;
    boundary_rom[324]=18'h3ffff;
    boundary_rom[325]=18'h3ffff;
    boundary_rom[326]=18'h3ffff;
    boundary_rom[327]=18'h3ffff;
    boundary_rom[328]=18'h3ffff;
    boundary_rom[329]=18'h3ffff;
    boundary_rom[330]=18'h3ffff;
    boundary_rom[331]=18'h3ffff;
    boundary_rom[332]=18'h3ffff;
    boundary_rom[333]=18'h3ffff;
    boundary_rom[334]=18'h3ffff;
    boundary_rom[335]=18'h3ffff;
    boundary_rom[336]=18'h3ffff;
    boundary_rom[337]=18'h3ffff;
    boundary_rom[338]=18'h3ffff;
    boundary_rom[339]=18'h3ffff;
    boundary_rom[340]=18'h3ffff;
    boundary_rom[341]=18'h3ffff;
    boundary_rom[342]=18'h3ffff;
    boundary_rom[343]=18'h3ffff;
    boundary_rom[344]=18'h3ffff;
    boundary_rom[345]=18'h3ffff;
    boundary_rom[346]=18'h3ffff;
    boundary_rom[347]=18'h3ffff;
    boundary_rom[348]=18'h3ffff;
    boundary_rom[349]=18'h3ffff;
    boundary_rom[350]=18'h3ffff;
    boundary_rom[351]=18'h3ffff;
    boundary_rom[352]=18'h3ffff;
    boundary_rom[353]=18'h3ffff;
    boundary_rom[354]=18'h3ffff;
    boundary_rom[355]=18'h3ffff;
    boundary_rom[356]=18'h3ffff;
    boundary_rom[357]=18'h3ffff;
    boundary_rom[358]=18'h3ffff;
    boundary_rom[359]=18'h3ffff;
    boundary_rom[360]=18'h0adff;
    boundary_rom[361]=18'h0afff;
    boundary_rom[362]=18'h0b1ff;
    boundary_rom[363]=18'h0b1ff;
    boundary_rom[364]=18'h0b3ff;
    boundary_rom[365]=18'h0b5ff;
    boundary_rom[366]=18'h0b5ff;
    boundary_rom[367]=18'h0b7ff;
    boundary_rom[368]=18'h0b9ff;
    boundary_rom[369]=18'h0bbff;
    boundary_rom[370]=18'h0bbff;
    boundary_rom[371]=18'h0bdff;
    boundary_rom[372]=18'h0bfff;
    boundary_rom[373]=18'h0bfff;
    boundary_rom[374]=18'h0c1ff;
    boundary_rom[375]=18'h0c3ff;
    boundary_rom[376]=18'h0c3ff;
    boundary_rom[377]=18'h0c5ff;
    boundary_rom[378]=18'h0c7ff;
    boundary_rom[379]=18'h0c9ff;
    boundary_rom[380]=18'h0c9ff;
    boundary_rom[381]=18'h0cbff;
    boundary_rom[382]=18'h0cdff;
    boundary_rom[383]=18'h0cdff;
    boundary_rom[384]=18'h0cfff;
    boundary_rom[385]=18'h0d1ff;
    boundary_rom[386]=18'h0d1ff;
    boundary_rom[387]=18'h0d3ff;
    boundary_rom[388]=18'h0d5ff;
    boundary_rom[389]=18'h0d7ff;
    boundary_rom[390]=18'h0d7ff;
    boundary_rom[391]=18'h0d9ff;
    boundary_rom[392]=18'h0dbff;
    boundary_rom[393]=18'h0dbff;
    boundary_rom[394]=18'h0ddff;
    boundary_rom[395]=18'h0dfff;
    boundary_rom[396]=18'h0dfff;
    boundary_rom[397]=18'h0e1ff;
    boundary_rom[398]=18'h0e3ff;
    boundary_rom[399]=18'h0e5ff;
    boundary_rom[400]=18'h0e5ff;
    boundary_rom[401]=18'h0e7ff;
    boundary_rom[402]=18'h0e9ff;
    boundary_rom[403]=18'h0e9ff;
    boundary_rom[404]=18'h0ebff;
    boundary_rom[405]=18'h0edff;
    boundary_rom[406]=18'h0edff;
    boundary_rom[407]=18'h0efff;
    boundary_rom[408]=18'h0f1ff;
    boundary_rom[409]=18'h0f3ff;
    boundary_rom[410]=18'h0f3ff;
    boundary_rom[411]=18'h0f5ff;
    boundary_rom[412]=18'h0f7ff;
    boundary_rom[413]=18'h0f7ff;
    boundary_rom[414]=18'h0f9ff;
    boundary_rom[415]=18'h0fbff;
    boundary_rom[416]=18'h0fbff;
    boundary_rom[417]=18'h0fdff;
    boundary_rom[418]=18'h0ffff;
    boundary_rom[419]=18'h101ff;
    boundary_rom[420]=18'h101ff;
    boundary_rom[421]=18'h103ff;
    boundary_rom[422]=18'h105ff;
    boundary_rom[423]=18'h105ff;
    boundary_rom[424]=18'h107ff;
    boundary_rom[425]=18'h109ff;
    boundary_rom[426]=18'h109ff;
    boundary_rom[427]=18'h10bff;
    boundary_rom[428]=18'h10dff;
    boundary_rom[429]=18'h10fff;
    boundary_rom[430]=18'h10fff;
    boundary_rom[431]=18'h111ff;
    boundary_rom[432]=18'h113ff;
    boundary_rom[433]=18'h113ff;
    boundary_rom[434]=18'h115ff;
    boundary_rom[435]=18'h117ff;
    boundary_rom[436]=18'h117ff;
    boundary_rom[437]=18'h119ff;
    boundary_rom[438]=18'h11bff;
    boundary_rom[439]=18'h11dff;
    boundary_rom[440]=18'h11dff;
    boundary_rom[441]=18'h11fff;
    boundary_rom[442]=18'h121ff;
    boundary_rom[443]=18'h121ff;
    boundary_rom[444]=18'h123ff;
    boundary_rom[445]=18'h125ff;
    boundary_rom[446]=18'h125ff;
    boundary_rom[447]=18'h127ff;
    boundary_rom[448]=18'h12987;
    boundary_rom[449]=18'h12b86;
    boundary_rom[450]=18'h12b86;
    boundary_rom[451]=18'h12d85;
    boundary_rom[452]=18'h12f84;
    boundary_rom[453]=18'h12f84;
    boundary_rom[454]=18'h13183;
    boundary_rom[455]=18'h13382;
    boundary_rom[456]=18'h13382;
    boundary_rom[457]=18'h13581;
    boundary_rom[458]=18'h13780;
    boundary_rom[459]=18'h1397f;
    boundary_rom[460]=18'h1397f;
    boundary_rom[461]=18'h13b7e;
    boundary_rom[462]=18'h13d7d;
    boundary_rom[463]=18'h13d7d;
    boundary_rom[464]=18'h13f7c;
    boundary_rom[465]=18'h1417b;
    boundary_rom[466]=18'h1417b;
    boundary_rom[467]=18'h1437a;
    boundary_rom[468]=18'h14579;
    boundary_rom[469]=18'h14778;
    boundary_rom[470]=18'h14778;
    boundary_rom[471]=18'h14977;
    boundary_rom[472]=18'h14b76;
    boundary_rom[473]=18'h14b76;
    boundary_rom[474]=18'h14d75;
    boundary_rom[475]=18'h14f74;
    boundary_rom[476]=18'h14f74;
    boundary_rom[477]=18'h15173;
    boundary_rom[478]=18'h15372;
    boundary_rom[479]=18'h15571;
    boundary_rom[480]=18'h15571;
    boundary_rom[481]=18'h15770;
    boundary_rom[482]=18'h1596f;
    boundary_rom[483]=18'h1596f;
    boundary_rom[484]=18'h15b6e;
    boundary_rom[485]=18'h15d6d;
    boundary_rom[486]=18'h15d6d;
    boundary_rom[487]=18'h15f6c;
    boundary_rom[488]=18'h1616b;
    boundary_rom[489]=18'h1636a;
    boundary_rom[490]=18'h1636a;
    boundary_rom[491]=18'h16569;
    boundary_rom[492]=18'h16768;
    boundary_rom[493]=18'h16768;
    boundary_rom[494]=18'h16967;
    boundary_rom[495]=18'h16b66;
    boundary_rom[496]=18'h16b66;
    boundary_rom[497]=18'h16d65;
    boundary_rom[498]=18'h16f64;
    boundary_rom[499]=18'h17163;
    boundary_rom[500]=18'h17163;
    boundary_rom[501]=18'h17362;
    boundary_rom[502]=18'h17561;
    boundary_rom[503]=18'h17561;
    boundary_rom[504]=18'h17760;
    boundary_rom[505]=18'h1795f;
    boundary_rom[506]=18'h1795f;
    boundary_rom[507]=18'h17b5e;
    boundary_rom[508]=18'h17d5d;
    boundary_rom[509]=18'h17f5c;
    boundary_rom[510]=18'h17f5c;
    boundary_rom[511]=18'h1815b;
    boundary_rom[512]=18'h1835a;
    boundary_rom[513]=18'h1835a;
    boundary_rom[514]=18'h18559;
    boundary_rom[515]=18'h18758;
    boundary_rom[516]=18'h18758;
    boundary_rom[517]=18'h18957;
    boundary_rom[518]=18'h18b56;
    boundary_rom[519]=18'h18d55;
    boundary_rom[520]=18'h18d55;
    boundary_rom[521]=18'h18f54;
    boundary_rom[522]=18'h19153;
    boundary_rom[523]=18'h19153;
    boundary_rom[524]=18'h19352;
    boundary_rom[525]=18'h19552;
    boundary_rom[526]=18'h19552;
    boundary_rom[527]=18'h19552;
    boundary_rom[528]=18'h19552;
    boundary_rom[529]=18'h19552;
    boundary_rom[530]=18'h19552;
    boundary_rom[531]=18'h19552;
    boundary_rom[532]=18'h19552;
    boundary_rom[533]=18'h19552;
    boundary_rom[534]=18'h19552;
    boundary_rom[535]=18'h19552;
    boundary_rom[536]=18'h19552;
    boundary_rom[537]=18'h19552;
    boundary_rom[538]=18'h19552;
    boundary_rom[539]=18'h19552;
    boundary_rom[540]=18'h19552;
    boundary_rom[541]=18'h19552;
    boundary_rom[542]=18'h19552;
    boundary_rom[543]=18'h19552;
    boundary_rom[544]=18'h19552;
    boundary_rom[545]=18'h19552;
    boundary_rom[546]=18'h19552;
    boundary_rom[547]=18'h19552;
    boundary_rom[548]=18'h19552;
    boundary_rom[549]=18'h19552;
    boundary_rom[550]=18'h19552;
    boundary_rom[551]=18'h19552;
    boundary_rom[552]=18'h19552;
    boundary_rom[553]=18'h19552;
    boundary_rom[554]=18'h19552;
    boundary_rom[555]=18'h19552;
    boundary_rom[556]=18'h19552;
    boundary_rom[557]=18'h19552;
    boundary_rom[558]=18'h19552;
    boundary_rom[559]=18'h19552;
    boundary_rom[560]=18'h19552;
    boundary_rom[561]=18'h19552;
    boundary_rom[562]=18'h19552;
    boundary_rom[563]=18'h19552;
    boundary_rom[564]=18'h19552;
    boundary_rom[565]=18'h19552;
    boundary_rom[566]=18'h19552;
    boundary_rom[567]=18'h19552;
    boundary_rom[568]=18'h19552;
    boundary_rom[569]=18'h19552;
    boundary_rom[570]=18'h19552;
    boundary_rom[571]=18'h19552;
    boundary_rom[572]=18'h19552;
    boundary_rom[573]=18'h19552;
    boundary_rom[574]=18'h19552;
    boundary_rom[575]=18'h19552;
    boundary_rom[576]=18'h19552;
    boundary_rom[577]=18'h19552;
    boundary_rom[578]=18'h19552;
    boundary_rom[579]=18'h19552;
    boundary_rom[580]=18'h19552;
    boundary_rom[581]=18'h19552;
    boundary_rom[582]=18'h19552;
    boundary_rom[583]=18'h19552;
    boundary_rom[584]=18'h19552;
    boundary_rom[585]=18'h19552;
    boundary_rom[586]=18'h19552;
    boundary_rom[587]=18'h19552;
    boundary_rom[588]=18'h19552;
    boundary_rom[589]=18'h19552;
    boundary_rom[590]=18'h19552;
    boundary_rom[591]=18'h19552;
    boundary_rom[592]=18'h19552;
    boundary_rom[593]=18'h19552;
    boundary_rom[594]=18'h19552;
    boundary_rom[595]=18'h19552;
    boundary_rom[596]=18'h19552;
    boundary_rom[597]=18'h19552;
    boundary_rom[598]=18'h19552;
    boundary_rom[599]=18'h19552;
    boundary_rom[600]=18'h19552;
    boundary_rom[601]=18'h19552;
    boundary_rom[602]=18'h19552;
    boundary_rom[603]=18'h19552;
    boundary_rom[604]=18'h19552;
    boundary_rom[605]=18'h19552;
    boundary_rom[606]=18'h19552;
    boundary_rom[607]=18'h19552;
    boundary_rom[608]=18'h19552;
    boundary_rom[609]=18'h19552;
    boundary_rom[610]=18'h19552;
    boundary_rom[611]=18'h19552;
    boundary_rom[612]=18'h19552;
    boundary_rom[613]=18'h19552;
    boundary_rom[614]=18'h19552;
    boundary_rom[615]=18'h19552;
    boundary_rom[616]=18'h19552;
    boundary_rom[617]=18'h19552;
    boundary_rom[618]=18'h19552;
    boundary_rom[619]=18'h19552;
    boundary_rom[620]=18'h19552;
    boundary_rom[621]=18'h19552;
    boundary_rom[622]=18'h19552;
    boundary_rom[623]=18'h19552;
    boundary_rom[624]=18'h19552;
    boundary_rom[625]=18'h19552;
    boundary_rom[626]=18'h19552;
    boundary_rom[627]=18'h19552;
    boundary_rom[628]=18'h19552;
    boundary_rom[629]=18'h19552;
    boundary_rom[630]=18'h19552;
    boundary_rom[631]=18'h19552;
    boundary_rom[632]=18'h19552;
    boundary_rom[633]=18'h19552;
    boundary_rom[634]=18'h19552;
    boundary_rom[635]=18'h19552;
    boundary_rom[636]=18'h19552;
    boundary_rom[637]=18'h19552;
    boundary_rom[638]=18'h19552;
    boundary_rom[639]=18'h19552;
    boundary_rom[640]=18'h19552;
    boundary_rom[641]=18'h19552;
    boundary_rom[642]=18'h19552;
    boundary_rom[643]=18'h19552;
    boundary_rom[644]=18'h19552;
    boundary_rom[645]=18'h19552;
    boundary_rom[646]=18'h19552;
    boundary_rom[647]=18'h19552;
    boundary_rom[648]=18'h19552;
    boundary_rom[649]=18'h19552;
    boundary_rom[650]=18'h19552;
    boundary_rom[651]=18'h19552;
    boundary_rom[652]=18'h19552;
    boundary_rom[653]=18'h19552;
    boundary_rom[654]=18'h19552;
    boundary_rom[655]=18'h19552;
    boundary_rom[656]=18'h19552;
    boundary_rom[657]=18'h19552;
    boundary_rom[658]=18'h19552;
    boundary_rom[659]=18'h19552;
    boundary_rom[660]=18'h19552;
    boundary_rom[661]=18'h19552;
    boundary_rom[662]=18'h19552;
    boundary_rom[663]=18'h19552;
    boundary_rom[664]=18'h19552;
    boundary_rom[665]=18'h19552;
    boundary_rom[666]=18'h19552;
    boundary_rom[667]=18'h19552;
    boundary_rom[668]=18'h19552;
    boundary_rom[669]=18'h19552;
    boundary_rom[670]=18'h19552;
    boundary_rom[671]=18'h19552;
    boundary_rom[672]=18'h19552;
    boundary_rom[673]=18'h19552;
    boundary_rom[674]=18'h19552;
    boundary_rom[675]=18'h19552;
    boundary_rom[676]=18'h19552;
    boundary_rom[677]=18'h19552;
    boundary_rom[678]=18'h19552;
    boundary_rom[679]=18'h19552;
    boundary_rom[680]=18'h19552;
    boundary_rom[681]=18'h19552;
    boundary_rom[682]=18'h19552;
    boundary_rom[683]=18'h19552;
    boundary_rom[684]=18'h19552;
    boundary_rom[685]=18'h19552;
    boundary_rom[686]=18'h19552;
    boundary_rom[687]=18'h19552;
    boundary_rom[688]=18'h19552;
    boundary_rom[689]=18'h19552;
    boundary_rom[690]=18'h19552;
    boundary_rom[691]=18'h19552;
    boundary_rom[692]=18'h19552;
    boundary_rom[693]=18'h19552;
    boundary_rom[694]=18'h19552;
    boundary_rom[695]=18'h19552;
    boundary_rom[696]=18'h19552;
    boundary_rom[697]=18'h19552;
    boundary_rom[698]=18'h19552;
    boundary_rom[699]=18'h19552;
    boundary_rom[700]=18'h19552;
    boundary_rom[701]=18'h19552;
    boundary_rom[702]=18'h19552;
    boundary_rom[703]=18'h19552;
    boundary_rom[704]=18'h19552;
    boundary_rom[705]=18'h19552;
    boundary_rom[706]=18'h19552;
    boundary_rom[707]=18'h19552;
    boundary_rom[708]=18'h19552;
    boundary_rom[709]=18'h19552;
    boundary_rom[710]=18'h19552;
    boundary_rom[711]=18'h19552;
    boundary_rom[712]=18'h19552;
    boundary_rom[713]=18'h19552;
    boundary_rom[714]=18'h19552;
    boundary_rom[715]=18'h19552;
    boundary_rom[716]=18'h19552;
    boundary_rom[717]=18'h19552;
    boundary_rom[718]=18'h19552;
    boundary_rom[719]=18'h19552;
end
// END GENERATED ROAD BOUNDS
reg [17:0] boundary_q;

function [29:0] blend64;
    input [29:0] a,b;
    input [6:0] w;
    reg [16:0] r,g,bl;
    begin
        r=a[29:20]*(7'd64-w)+b[29:20]*w+17'd32;
        g=a[19:10]*(7'd64-w)+b[19:10]*w+17'd32;
        bl=a[9:0]*(7'd64-w)+b[9:0]*w+17'd32;
        blend64={r[15:6],g[15:6],bl[15:6]};
    end
endfunction
function [29:0] smooth121;
    input [29:0] a,b,c;
    reg [11:0] r,g,bl;
    begin
        r={2'd0,a[29:20]}+({2'd0,b[29:20]}<<1)+{2'd0,c[29:20]}+12'd2;
        g={2'd0,a[19:10]}+({2'd0,b[19:10]}<<1)+{2'd0,c[19:10]}+12'd2;
        bl={2'd0,a[9:0]}+({2'd0,b[9:0]}<<1)+{2'd0,c[9:0]}+12'd2;
        smooth121={r[11:2],g[11:2],bl[11:2]};
    end
endfunction
function [3:0] threshold4;
    input [9:0] x,y;
    reg [31:0] h;
    begin
        h={22'd0,x}+({22'd0,y}<<10)+32'h7531;
        h=h^(h<<13); h=h^(h>>17); h=h^(h<<5);
        h=h+(h<<7); h=h^(h>>9); h=h+(h<<3); h=h^(h>>11);
        threshold4=h[5:2];
    end
endfunction
function [15:0] quantize;
    input [29:0] q;
    input [3:0] th;
    reg [5:0] r,b;
    reg [6:0] g;
    begin
        r={1'b0,q[28:24]}+((q[23:20]>th) ? 6'd1 : 6'd0);
        g={1'b0,q[19:14]}+((q[13:10]>th) ? 7'd1 : 7'd0);
        b={1'b0,q[8:4]}+((q[3:0]>th) ? 6'd1 : 6'd0);
        quantize={(r>31 ? 5'd31 : r[4:0]),
                  (g>63 ? 6'd63 : g[5:0]),(b>31 ? 5'd31 : b[4:0])};
    end
endfunction

// Per-query pipeline: read controls -> y blend -> x blend -> edge blend ->
// quantize. Runs during the existing 32-cycle perspective division latency.
wire [9:0] ydelta=(query_y<16) ? 10'd0 : query_y-10'd16;
wire [4:0] yi0=ydelta[9:5];
wire [4:0] yi1=(yi0>=22) ? 5'd22 : yi0+1'b1;
wire [9:0] xdelta=(query_x<90) ? 10'd0 :
                       (query_x>=482) ? 10'd391 : query_x-10'd90;
wire [13:0] xscaled=({4'd0,xdelta}<<4)+({4'd0,xdelta}<<2)+{4'd0,xdelta}+14'd64;
wire [6:0] xweight=(xscaled[13:7]>64) ? 7'd64 : xscaled[13:7];
reg [29:0] l0,l1,r0,r1,left_y,right_y,field_xy,edge_y,field_edge;
reg [6:0] yw0,xw0,xw1,ew1,ew2;
reg edge_is_right;
reg [3:0] th0,th1,th2,th3;
reg v0,v1,v2,v3;
reg [3:0] query_pipe;
reg [9:0] query_x_q;
wire [9:0] left_edge={1'b0,boundary_q[17:9]}+10'd90;
wire [9:0] right_edge={1'b0,boundary_q[8:0]}+10'd90;
wire left_near=(boundary_q[17:9]!=9'h1ff) && (query_x_q>left_edge) &&
               ((query_x_q-left_edge)<32);
wire right_near=(boundary_q[8:0]!=9'h1ff) && (query_x_q<right_edge) &&
                ((right_edge-query_x_q)<32);
wire choose_right=right_near && (!left_near ||
                      ((right_edge-query_x_q)<(query_x_q-left_edge)));
wire [9:0] edge_dist=choose_right ? right_edge-query_x_q : query_x_q-left_edge;
wire [6:0] edge_weight=(left_near || right_near) ? (7'd64-{edge_dist[5:0],1'b0}) : 7'd0;

// Use measured RGB in the real 16-pixel strip beside the hole, without a
// brightness/chroma classifier. Bin averaging and 1:2:1 spatial smoothing
// reduce noise without imposing a synthetic road colour.
wire sample_left=(sample_x<360);
wire [9:0] sample_edge=sample_left ? left_edge:right_edge;
wire edge_exists=sample_left ? (boundary_q[17:9]!=9'h1ff) : (boundary_q[8:0]!=9'h1ff);
wire in_strip=sample_left ? ((sample_x<=sample_edge) && ((sample_edge-sample_x)<16)) :
                           ((sample_x>=sample_edge) && ((sample_x-sample_edge)<16));
wire [5:0] sample_index={1'b0,sample_y[9:5]}+(sample_left ? 6'd0:6'd23);
assign edge_sample_valid=sample_valid && collecting && !busy &&
    (sample_x>=90) && (sample_x<482) &&
    (sample_y[1:0]==0) && (sample_y<720) && edge_exists && in_strip &&
    (count[sample_index]<256);

// One short iterative divider is shared by all frame statistics and missing
// control interpolation weights. It never sits on the pixel data path.
reg div_start;
reg [17:0] dividend,divisor;
wire div_done;
wire [17:0] quotient;
bev_unsigned_divider #(.WIDTH(18)) u_profile_divider(
    .clk(clk),.rst_n(rst_n),.start(div_start),.dividend(dividend),
    .divisor(divisor),.done(div_done),.quotient(quotient)
);
reg [5:0] slot,scan_base,prev_slot,next_slot;
reg [4:0] scan_bin;
reg [1:0] channel;
reg prev_found,next_found,other_side;
wire [4:0] slot_bin=(slot<23) ? slot[4:0] : slot-6'd23;
wire [5:0] own_base=(slot<23) ? 6'd0:6'd23;
wire [5:0] scan_slot=scan_base+{1'b0,scan_bin};
wire [4:0] prev_bin=(prev_slot<23) ? prev_slot[4:0]:prev_slot-6'd23;
wire [4:0] next_bin=(next_slot<23) ? next_slot[4:0]:next_slot-6'd23;
wire [4:0] span=next_bin-prev_bin;
wire [4:0] distance_bin=slot_bin-prev_bin;
reg [29:0] chosen_q, smooth_a,smooth_b,smooth_c;
wire [29:0] mean_selected=mean_q[slot];
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        query_pipe<=0; fill_valid<=0; fill_pixel<=0;
        v0<=0;v1<=0;v2<=0;v3<=0;
    end else begin
        query_pipe<={query_pipe[2:0],query_start};
        if (query_start) begin
            boundary_q<=boundary_rom[query_y];
            query_x_q<=query_x;
            l0<=active_q[yi0]; l1<=active_q[yi1];
            r0<=active_q[{1'b0,yi0}+6'd23];
            r1<=active_q[{1'b0,yi1}+6'd23];
            yw0<={1'b0,ydelta[4:0],1'b0};
            xw0<=xweight;
            th0<=threshold4(query_x,query_y);
            v0<=active_valid && (active_ready==source_ready);
        end
        if (query_pipe[0]) begin
            left_y<=blend64(l0,l1,yw0); right_y<=blend64(r0,r1,yw0);
            xw1<=xw0; ew1<=edge_weight; edge_is_right<=choose_right;
            th1<=th0; v1<=v0;
        end
        if (query_pipe[1]) begin
            field_xy<=blend64(left_y,right_y,xw1);
            edge_y<=edge_is_right ? right_y:left_y;
            ew2<=ew1; th2<=th1; v2<=v1;
        end
        if (query_pipe[2]) begin
            field_edge<=blend64(field_xy,edge_y,ew2);
            th3<=th2; v3<=v2;
        end
        if (query_pipe[3]) begin
            fill_pixel<=quantize(field_edge,th3); fill_valid<=v3;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state<=IDLE; active_valid<=0; collecting<=0; epoch_bad<=0;
        epoch_ready<=0;active_ready<=0; observed<=0;side_has_data<=0;
        slot<=0;channel<=0;div_start<=0;dividend<=0;divisor<=1;
        scan_base<=0;scan_bin<=0;prev_found<=0;next_found<=0;
        prev_slot<=0;next_slot<=0;other_side<=0;chosen_q<=0;
        for(i=0;i<TOTAL;i=i+1) count[i]<=0;
    end else begin
        div_start<=0;
        if (collecting && (source_ready!=epoch_ready)) epoch_bad<=1;
        if (frame_begin && !busy) begin
            collecting<=1; epoch_ready<=source_ready; epoch_bad<=0;
            for(i=0;i<TOTAL;i=i+1) count[i]<=0;
        end
        if (edge_sample_valid) begin
                if (count[sample_index]==0) begin
                    sum_r[sample_index]<={9'd0,sample_pixel[15:11]};
                    sum_g[sample_index]<={8'd0,sample_pixel[10:5]};
                    sum_b[sample_index]<={9'd0,sample_pixel[4:0]};
                end else begin
                    sum_r[sample_index]<=sum_r[sample_index]+sample_pixel[15:11];
                    sum_g[sample_index]<=sum_g[sample_index]+sample_pixel[10:5];
                    sum_b[sample_index]<=sum_b[sample_index]+sample_pixel[4:0];
                end
                count[sample_index]<=count[sample_index]+1'b1;
        end
        case(state)
        IDLE: if(frame_end) begin
            collecting<=0; active_valid<=0; observed<=0;
            side_has_data<=0; slot<=0; channel<=0; state<=N_CHECK;
        end
        N_CHECK: begin
            channel<=0;
            if(count[slot]>=8) begin
                observed[slot]<=1;
                if(slot<23) side_has_data[0]<=1; else side_has_data[1]<=1;
                state<=N_DIV;
            end else begin
                mean_q[slot]<=0;
                if(slot==TOTAL-1) begin slot<=0;state<=FIND_INIT;end
                else slot<=slot+1'b1;
            end
        end
        N_DIV: begin
            case(channel)
            0: dividend<={sum_r[slot],4'd0}+({9'd0,count[slot]}>>1);
            1: dividend<={sum_g[slot],4'd0}+({9'd0,count[slot]}>>1);
            default: dividend<={sum_b[slot],4'd0}+({9'd0,count[slot]}>>1);
            endcase
            divisor<={9'd0,count[slot]};div_start<=1;state<=N_WAIT;
        end
        N_WAIT: if(div_done) begin
            case(channel)
            0:mean_q[slot][29:20]<=quotient[9:0];
            1:mean_q[slot][19:10]<=quotient[9:0];
            default:mean_q[slot][9:0]<=quotient[9:0];
            endcase
            if(channel==2) begin
                if(slot==TOTAL-1) begin slot<=0;state<=FIND_INIT;end
                else begin slot<=slot+1'b1;state<=N_CHECK;end
            end else begin channel<=channel+1'b1;state<=N_DIV;end
        end
        FIND_INIT: begin
            scan_base<=own_base;scan_bin<=0;
            prev_found<=0;next_found<=0;other_side<=0;state<=FIND_SCAN;
        end
        FIND_SCAN: begin
            if(observed[scan_slot]) begin
                if(scan_bin<=slot_bin) begin prev_slot<=scan_slot;prev_found<=1;end
                if((scan_bin>=slot_bin) && !next_found) begin next_slot<=scan_slot;next_found<=1;end
            end
            if(scan_bin==22) state<=FIND_PICK;
            else scan_bin<=scan_bin+1'b1;
        end
        FIND_PICK: begin
            if(prev_found && next_found && (prev_slot!=next_slot)) begin
                dividend<=({13'd0,distance_bin}<<6)+{13'd0,(span>>1)};
                divisor<={13'd0,span};div_start<=1;state<=WEIGHT_WAIT;
            end else if(prev_found || next_found) begin
                chosen_q<=mean_q[prev_found ? prev_slot:next_slot];state<=FILL_SLOT;
            end else if(!other_side) begin
                scan_base<=(own_base==0) ? 6'd23:6'd0;
                scan_bin<=0;prev_found<=0;next_found<=0;other_side<=1;state<=FIND_SCAN;
            end else begin chosen_q<=30'd0;state<=FILL_SLOT;end
        end
        WEIGHT_WAIT: if(div_done) begin
            chosen_q<=blend64(mean_q[prev_slot],mean_q[next_slot],quotient[6:0]);
            state<=FILL_SLOT;
        end
        FILL_SLOT: begin
            // Only originally observed controls participate in searches.
            // Filling missing controls in-place cannot contaminate donors.
            mean_q[slot]<=chosen_q;
            if(slot==TOTAL-1) begin slot<=0;state<=SMOOTH_READ;end
            else begin slot<=slot+1'b1;state<=FIND_INIT;end
        end
        SMOOTH_READ: begin
            smooth_a<=mean_q[(slot_bin==0) ? slot:slot-1'b1];
            smooth_b<=mean_q[slot];
            smooth_c<=mean_q[(slot_bin==22) ? slot:slot+1'b1];
            state<=SMOOTH_WRITE;
        end
        SMOOTH_WRITE: begin
            active_q[slot]<=smooth121(smooth_a,smooth_b,smooth_c);
            if(slot==TOTAL-1) state<=PUBLISH;
            else begin slot<=slot+1'b1;state<=SMOOTH_READ;end
        end
        PUBLISH: begin
            active_ready<=epoch_ready;
            active_valid<=(|side_has_data) && !epoch_bad && (source_ready==epoch_ready);
            state<=IDLE;
        end
        default:state<=IDLE;
        endcase
    end
end
endmodule
