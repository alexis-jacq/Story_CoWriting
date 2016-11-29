from rest_framework import serializers
from .models import *

class AccessoriesSerializer(serializers.ModelSerializer):
   
    class Meta:
        model = Accessories
        # fields = ('','')
        fields = '__all__'

class CategoriesSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Categories
        # fields = ('','')
        fields = '__all__'


class ChAcSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = ChAc
        # fields = ('','')
        fields = '__all__'


class CharactersSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Characters
        # fields = ('','')
        fields = '__all__'

class ContainersSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Containers
        # fields = ('','')
        fields = '__all__'


class DrinksSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Drinks
        # fields = ('','')
        fields = '__all__'


class EnemiesSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Enemies
        # fields = ('','')
        fields = '__all__'


class FoodsSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Foods
        # fields = ('','')
        fields = '__all__'


class FriendsSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Friends
        # fields = ('','')
        fields = '__all__'


class JobSerializer(serializers.ModelSerializer):
   
    class Meta:
        model = Job
        # fields = ('','')
        fields = '__all__'


class RoleSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Role
        # fields = ('','')
        fields = '__all__'


class ScenariosSerializer(serializers.ModelSerializer):

    class Meta:
        model = Scenarios
        # fields = ('','')
        fields = '__all__'


class SpeciesSerializer(serializers.ModelSerializer):
    
    class Meta:
        model = Species
        # fields = ('','')
        fields = '__all__'


class StChSerializer(serializers.ModelSerializer):

    class Meta:
        mmodel = StCh
        # fields = ('','')
        fields = '__all__'


class StorySerializer(serializers.ModelSerializer):

    class Meta:
       model = Story
       # fields = ('','')
       fields = '__all__'


class ToolsSerializer(serializers.ModelSerializer):

    class Meta:
        model = Tools
        # fields = ('','')
        fields = '__all__'


class TypeSerializer(serializers.ModelSerializer):

    class Meta:
        model = Type
        # fields = ('','')
        fields = '__all__'

class FullStSerializer(serializers.ModelSerializer):

    class Meta:
        model = FullSt
        # fields = ('','')
        fields = '__all__'

class GenderSerializer(serializers.ModelSerializer):

    class Meta:
        model = Gender
        # fields = ('','')
        fields = '__all__'