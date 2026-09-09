extends SceneTree
func _init() -> void:
 for key in ClassDB.class_get_integer_constant_list("Environment"):
  if str(key).contains("REFLECT"):
   print(key,"=",ClassDB.class_get_integer_constant("Environment",key))
 quit()
