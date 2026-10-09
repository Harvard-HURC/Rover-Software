import sys
from PIL import Image, ImageDraw
from pathlib import Path
from textures_font import font
def sheet(out, tags, names=('chase','onboard','aerial','ground'), W=640, labels=None):
    rows=[]
    for tag in tags:
        ims=[]
        for n in names:
            p=Path('out')/tag/f'{n}.png'
            im=Image.open(p).convert('RGB') if p.exists() else Image.new('RGB',(W,360))
            ims.append(im.resize((W,int(im.height*W/im.width))))
        h=max(i.height for i in ims)
        row=Image.new('RGB',(W*len(ims),h))
        for k,i in enumerate(ims): row.paste(i,(k*W,0))
        d=ImageDraw.Draw(row); d.rectangle([0,0,10+12*len(tag),24],fill=(0,0,0)); d.text((5,4),tag,fill=(255,255,255),font=font(16))
        rows.append(row)
    S=Image.new('RGB',(rows[0].width,sum(r.height for r in rows)))
    y=0
    for r in rows: S.paste(r,(0,y)); y+=r.height
    S.save(out,quality=88)
if __name__=='__main__':
    sheet(sys.argv[1], sys.argv[2:])
