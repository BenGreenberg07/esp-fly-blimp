import re, pcbnew
NET="esp_fly_clone.net"; OUT="kicad/esp_fly_clone.kicad_pcb"
FPSTD="/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
XIAODIR="/Users/ben/OPL_Kicad_Library/Seeed Studio XIAO Series Library"
W,H=17.9,21.2
txt=open(NET).read()
comps={}
for m in re.finditer(r'\(comp\s*\(ref "([^"]+)"\)\s*\(value "([^"]+)"\).*?\(footprint "([^"]+)"\)', txt, re.S):
    lib,name=m.group(3).split(":",1); comps[m.group(1)]=[lib,name,m.group(2)]
pad_net={}; net_comps={}
for blk in re.split(r'\(net\s', txt.split("(nets",1)[1])[1:]:
    nm=re.search(r'\(name "([^"]+)"\)',blk)
    if not nm: continue
    n=nm.group(1)
    for ref,pad in re.findall(r'\(node\s*\(ref "([^"]+)"\)\s*\(pin "([^"]+)"',blk):
        pad_net[(ref,pad)]=n; net_comps.setdefault(n,set()).add(ref)
def cnets(ref): return {v for (r,p),v in pad_net.items() if r==ref}

# ---- assign positions/side ----
place={}  # ref -> (x,y,rot,back)
corners=[(3.2,3.4),(14.7,3.4),(3.2,17.8),(14.7,17.8)]
mpad=[(1.6,1.7,90),(16.3,1.7,90),(1.6,19.5,90),(16.3,19.5,90)]
Qs=sorted([r for r in comps if r.startswith('Q')]); Js=sorted([r for r in comps if r.startswith('J')])
for i,q in enumerate(Qs):
    qx,qy=corners[i]; place[q]=(qx,qy,0,True)
    g=[n for n in cnets(q)][0]
    # resistors on this MOSFET's gate net cluster
    gate_net=pad_net[(q,'1')]
    rs=[r for r in net_comps.get(gate_net,()) if r.startswith('R')]
    off=[(-2.0,0),(2.0,0)]
    for j,r in enumerate(rs[:2]):
        place[r]=(qx+off[j][0], qy+off[j][1],90,True)
# motor pads at corners (J1-J4 are the SolderWire ones: value Motor*)
mi=0
for jr in Js:
    if comps[jr][2].startswith('Motor'):
        x,y,rot=mpad[mi]; place[jr]=(x,y,rot,True); mi+=1
    else:
        place[jr]=(W/2,2.4,0,True)  # JST top-center back
# bulk caps (22u) bottom near JST
bulk=[r for r in comps if comps[r][0]=='Device' and comps[r][1].startswith('C_0805')]
for k,c in enumerate(bulk): place[c]=(7.0+k*3.9,4.2,0,True)
# white LEDs (0603) + their 150R -> bottom corners
wled=[r for r in comps if comps[r][1].startswith('LED_0603')]
for k,d in enumerate(wled):
    x=4.5 if k==0 else 13.4; place[d]=(x,19.6,0,True)
    an=pad_net[(d,'2')]
    for r in net_comps.get(an,()):
        if r.startswith('R'): place[r]=(x,18.0,90,True)
# TOP side: IMU center
imu=[r for r in comps if r.startswith('U') and comps[r][1].startswith('QFN')][0]
place[imu]=(W/2,10.6,0,False)
# 100n near imu; pullups; divider; status leds
used_top=[(11.2,10.6),(6.7,8.6),(6.7,12.6),(11.2,8.0),(11.2,13.2),
          (W/2,6.0),(W/2,15.2),(5.2,10.6),(6.0,6.0),(12.0,15.2),(6.0,15.2),(12.0,6.0)]
ti=0
for r in comps:
    if r in place: continue
    if r=='U1': continue
    x,y=used_top[ti%len(used_top)]; ti+=1; place[r]=(x,y,0,False)
# XIAO centered
place['U1']=(W/2,H/2,0,False)

# ---- build board ----
board=pcbnew.CreateEmptyBoard(); board.SetCopperLayerCount(4)
netmap={}
for n in sorted(set(pad_net.values())):
    ni=pcbnew.NETINFO_ITEM(board,n); board.Add(ni); netmap[n]=ni
def libdir(lib): return XIAODIR if lib=="XIAO" else f"{FPSTD}/{lib}.pretty"
for ref,(lib,name,val) in comps.items():
    fp=pcbnew.FootprintLoad(libdir(lib),name)
    if not fp: print("FAIL",ref,lib,name); continue
    fp.SetReference(ref); fp.SetValue(val)
    board.Add(fp)
    x,y,rot,back=place.get(ref,(W/2,H/2,0,False))
    fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x),pcbnew.FromMM(y)))
    if rot: fp.SetOrientationDegrees(rot)
    if back: fp.Flip(pcbnew.VECTOR2I(pcbnew.FromMM(x),pcbnew.FromMM(y)), False)
    for pad in fp.Pads():
        k=(ref,pad.GetNumber())
        if k in pad_net: pad.SetNet(netmap[pad_net[k]])
rect=pcbnew.PCB_SHAPE(board); rect.SetShape(pcbnew.SHAPE_T_RECT)
rect.SetStart(pcbnew.VECTOR2I(0,0)); rect.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(W),pcbnew.FromMM(H)))
rect.SetLayer(pcbnew.Edge_Cuts); rect.SetWidth(pcbnew.FromMM(0.15)); board.Add(rect)
pcbnew.SaveBoard(OUT,board)
print("SAVED",OUT,"| comps",len(comps))
