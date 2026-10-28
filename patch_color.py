p = "configs/srm.yaml"
s = open(p, encoding="utf-8").read()
s = s.replace('enemy_color: "red"', 'enemy_color: "blue"')
open(p, "w", encoding="utf-8").write(s)
print("OK")
