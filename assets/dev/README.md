## Making Animated Meshes

### Warnings
- name your Armature 'Armature' (case sensitive)
- only ONE mesh per blender file
- only ONE texture per mesh
- DO NOT include numbers in your bone names
- name your mesh 'mesh' (?)

### Steps
- make your mesh and texture it
- make sure you clear or apply all transforms
- shift + a in an armature & position bones in edit mode
- name your bones
- select mesh then armature and ctrl + p to join them 'with empty groups'
- manually set vertex groups in edit mode
- make sure keyframe 1 is default pose
- animate

### Keyframes

#### pistols [8]
- 0 : idle
- 1 : shoot
- 2 : mag out (slide back)
- 3 : mag in  (slide back)
- 4 : mag out (slide fwd )
- 5 : mag in  (slide fwd )
- 6 : inspect 1
- 7 : inspect 2

#### bolt action [8]
- 0 : idle
- 1 : fire
- 2 : bolt_up
- 3 : bolt_back
- 4 : bullet_in_anticipation
- 5 : bullet_in
- 6 : inspect 1
- 7 : inspect 2

#### clip loaded [10]
- 0 : idle
- 1 : fire
- 2 : slide back
- 3 : bolt_back
- 4 : bullet_in_anticipation
- 5 : bullet_in
- 6 : clip_in_anticipation
- 7 : clip_in
- 8 : clip_flick
- 9 : inspect 1
- 10: inspect 2

#### sub machine guns [8]
- 0 : idle
- 1 : shoot
- 2 : mag out (slide back)
- 3 : mag in  (slide back)
- 4 : mag out (slide fwd )
- 5 : mag in  (slide fwd )
- 6 : inspect 1
- 7 : inspect 2

#### light machine guns [8]
- 0 : idle
- 1 : shoot
- 2 : drum out
- 3 : drum in
- 6 : inspect 1
- 7 : inspect 2