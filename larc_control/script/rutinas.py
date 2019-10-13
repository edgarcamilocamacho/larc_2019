# Abrir pinza
pub_gripper.publish(0.65)
# Posicion inicial
go_to_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
go_to_pos([0.0, 0.0, 0.1, 1.2, 0.4, 1.57])

# Sobre bloque izquierdo
go_to_pos([-0.1, 0.055, -0.6, 1.0, -0.2, 1.47])
# Bajar al bloque
go_to_pos([-0.1, 0.055, -0.9, 1.0, -0.5, 1.47])
# Cerrar pinza
pub_gripper.publish(0.30)
# Subir bloque
go_to_pos([-0.1, 0.055, -0.6, 1.0, -0.2, 1.47])
# Ir a barco azul
go_to_pos([2.4, 0.16, -0.6, 1.0, -0.2, 0.85])
# Bajar en barzo azul
go_to_pos([2.4, 0.16, -1.9, -0.2, -0.20, 0.85])
# Abrir pinza
pub_gripper.publish(0.65)
# Subir en barco azul
go_to_pos([2.4, 0.16, -0.6, 1.0, -0.2, 0.85])
# Sobre los bloques
# go_to_pos([-0.1, 0.055, -0.6, 1.0, -0.2, 1.47])
# Sobre bloque derecho
go_to_pos([-0.2, 0.055, -0.6, 1.0, -0.2, -1.77])
# Baja al bloque
go_to_pos([-0.2, 0.055, -0.9, 1.0, -0.5, -1.77])
# Cierra la pinza
pub_gripper.publish(0.30)
# Sube el bloque
go_to_pos([-0.2, 0.055, -0.6, 1.0, -0.2, -1.77])
# Va al barco azul
go_to_pos([2.4, 0.16, -0.6, 1.0, -0.2, 0.85])
# Baja el bloque
go_to_pos([2.41, 0.14, -1.45, 0.4, -0.4, 0.85])
# Abre la pinza
pub_gripper.publish(0.65)
# Sube la pinza
go_to_pos([2.4, 0.16, -0.6, 1.0, -0.2, 0.85])
# Sobre los bloques
go_to_pos([-0.1, 0.055, -0.6, 1.0, -0.2, 1.47])




