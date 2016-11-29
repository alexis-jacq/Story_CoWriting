# This is an auto-generated Django model module.
# You'll have to do the following manually to clean this up:
#   * Rearrange models' order
#   * Make sure each model has one field with primary_key=True
#   * Make sure each ForeignKey has `on_delete` set to the desired behavior.
#   * Remove `managed = False` lines if you wish to allow Django to create, modify, and delete the table
# Feel free to rename the models, but don't rename db_table values or field names.
from __future__ import unicode_literals
from django.core.urlresolvers import reverse

from django.db import models

class FullSt(models.Model):
    """docstring for ClassName"""
    def __init__(self, arg):
        super(ClassName, self).__init__()
        self.arg = arg
    class Meta:
        managed = False
        
class Accessories(models.Model):
    idaccessories = models.AutoField(db_column='idAccessories', primary_key=True)  # Field name made lowercase.
    iddr = models.ForeignKey('Drinks', models.DO_NOTHING, db_column='idDr', blank=True, null=True)  # Field name made lowercase.
    idfo = models.ForeignKey('Foods', models.DO_NOTHING, db_column='idFo', blank=True, null=True)  # Field name made lowercase.
    idto = models.ForeignKey('Tools', models.DO_NOTHING, db_column='idTo', blank=True, null=True)  # Field name made lowercase.
    idwe = models.ForeignKey('Weapons', models.DO_NOTHING, db_column='idWe', blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Accessories'


class Categories(models.Model):
    idcategories = models.AutoField(db_column='idCategories', primary_key=True)  # Field name made lowercase.
    nameca = models.CharField(db_column='nameCa', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Categories'


class ChAc(models.Model):
    idch_ac = models.AutoField(db_column='idCh_Ac', primary_key=True)  # Field name made lowercase.
    idch = models.ForeignKey('Characters', models.DO_NOTHING, db_column='idCh')  # Field name made lowercase.
    idac = models.ForeignKey(Accessories, models.DO_NOTHING, db_column='idAc')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Ch_Ac'


class ChTi(models.Model):
    idch_ti = models.IntegerField(db_column='idCh_Ti', primary_key=True)  # Field name made lowercase.
    idch = models.ForeignKey('Characters', models.DO_NOTHING, db_column='idCh')  # Field name made lowercase.
    idti = models.ForeignKey('Ticks', models.DO_NOTHING, db_column='idTi')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Ch_Ti'


class Characters(models.Model):
    idcharacters = models.AutoField(db_column='idCharacters', primary_key=True)  # Field name made lowercase.
    namech = models.CharField(db_column='nameCh', max_length=45, blank=True, null=True)  # Field name made lowercase.
    idro = models.ForeignKey('Role', models.DO_NOTHING, db_column='idRo', blank=True, null=True)  # Field name made lowercase.
    idjo = models.ForeignKey('Job', models.DO_NOTHING, db_column='idJo', blank=True, null=True)  # Field name made lowercase.
    idsp = models.ForeignKey('Species', models.DO_NOTHING, db_column='idSp', blank=True, null=True)  # Field name made lowercase.
    idge = models.ForeignKey('Gender', models.DO_NOTHING, db_column='idGe', blank=True, null=True)  # Field name made lowercase.
    idda = models.ForeignKey('Dance', models.DO_NOTHING, db_column='idDa', blank=True, null=True)  # Field name made lowercase.
    idpl = models.ForeignKey('Place', models.DO_NOTHING, db_column='idPl', blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Characters'


class Containers(models.Model):
    idcontainers = models.AutoField(db_column='idContainers', primary_key=True)  # Field name made lowercase.
    nameco = models.CharField(db_column='nameCo', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Containers'


class Dance(models.Model):
    idda = models.AutoField(db_column='idDa', primary_key=True)  # Field name made lowercase.
    nameda = models.CharField(max_length=45, blank=True, null=True)

    class Meta:
        managed = False
        db_table = 'Dance'


class Drinks(models.Model):
    iddrinks = models.AutoField(db_column='idDrinks', primary_key=True)  # Field name made lowercase.
    namedr = models.CharField(db_column='nameDr', max_length=45, blank=True, null=True)  # Field name made lowercase.
    idco = models.ForeignKey(Containers, models.DO_NOTHING, db_column='idCo')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Drinks'


class Enemies(models.Model):
    idenemies = models.AutoField(db_column='idEnemies', primary_key=True)  # Field name made lowercase.
    idch = models.ForeignKey(Characters, models.DO_NOTHING, db_column='idCh')  # Field name made lowercase.
    idch_field = models.ForeignKey(Characters, models.DO_NOTHING, db_column='idCh_', related_name='+')  # Field name made lowercase. Field renamed because it ended with '_'.

    class Meta:
        managed = False
        db_table = 'Enemies'


class Foods(models.Model):
    idfoods = models.AutoField(db_column='idFoods', primary_key=True)  # Field name made lowercase.
    namefo = models.CharField(db_column='nameFo', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Foods'


class Friends(models.Model):
    idfriends = models.AutoField(db_column='idFriends', primary_key=True)  # Field name made lowercase.
    idch = models.ForeignKey(Characters, models.DO_NOTHING, db_column='idCh')  # Field name made lowercase.
    idch_field = models.ForeignKey(Characters, models.DO_NOTHING, db_column='idCh_', related_name='+')  # Field name made lowercase. Field renamed because it ended with '_'.

    class Meta:
        managed = False
        db_table = 'Friends'


class Gender(models.Model):
    idge = models.AutoField(db_column='idGe', primary_key=True)  # Field name made lowercase.
    namege = models.CharField(db_column='nameGe', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Gender'


class Job(models.Model):
    idjobs = models.AutoField(db_column='idJobs', primary_key=True)  # Field name made lowercase.
    namejo = models.CharField(db_column='nameJo', max_length=45, blank=True, null=True)  # Field name made lowercase.
    idge = models.ForeignKey(Gender, models.DO_NOTHING, db_column='idGe')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Job'


class Place(models.Model):
    idplace = models.AutoField(db_column='idPlace', primary_key=True)  # Field name made lowercase.
    namepl = models.CharField(max_length=45, blank=True, null=True)
    class Meta:
        managed = False
        db_table = 'Place'


class Role(models.Model):
    idrole = models.AutoField(db_column='idRole', primary_key=True)  # Field name made lowercase.
    namero = models.CharField(db_column='nameRo', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Role'


class Scenarios(models.Model):
    idscenarios = models.AutoField(db_column='idScenarios', primary_key=True)  # Field name made lowercase.
    namesc = models.CharField(max_length=45, blank=True, null=True)

    class Meta:
        managed = False
        db_table = 'Scenarios'


class Species(models.Model):
    idspecies = models.AutoField(db_column='idSpecies', primary_key=True)  # Field name made lowercase.
    namesp = models.CharField(db_column='nameSp', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Species'


class StCh(models.Model):
    idst_ch = models.AutoField(db_column='idSt_Ch', primary_key=True)  # Field name made lowercase.
    idst = models.ForeignKey('Story', models.DO_NOTHING, db_column='idSt')  # Field name made lowercase.
    idch = models.ForeignKey(Characters, models.DO_NOTHING, db_column='idCh')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'St_Ch'


class Story(models.Model):
    idstory = models.AutoField(db_column='idStory', primary_key=True)  # Field name made lowercase.
    namest = models.CharField(db_column='nameSt', max_length=45, blank=True, null=True)  # Field name made lowercase.
    script = models.CharField(max_length=45, blank=True, null=True)
    idty = models.ForeignKey('Type', models.DO_NOTHING, db_column='idTy')  # Field name made lowercase.
    idsc = models.ForeignKey(Scenarios, models.DO_NOTHING, db_column='idSc')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Story'


class Ticks(models.Model):
    idti = models.AutoField(db_column='idTi', primary_key=True)  # Field name made lowercase.
    nameti = models.CharField(db_column='nameti', max_length=45, blank=True, null=True)  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Ticks'


class Tools(models.Model):
    idtools = models.AutoField(db_column='idTools', primary_key=True)  # Field name made lowercase.
    nameto = models.CharField(db_column='nameTo', max_length=45, blank=True, null=True)  # Field name made lowercase.
    idca = models.ForeignKey(Categories, models.DO_NOTHING, db_column='idCa')  # Field name made lowercase.
    idjo = models.ForeignKey(Job, models.DO_NOTHING, db_column='idJo')  # Field name made lowercase.

    class Meta:
        managed = False
        db_table = 'Tools'


class Type(models.Model):
    idtype = models.AutoField(db_column='idType', primary_key=True)  # Field name made lowercase.
    namety = models.CharField(max_length=45, blank=True, null=True)

    class Meta:
        managed = False
        db_table = 'Type'


class Weapons(models.Model):
    idwe = models.AutoField(db_column='idWe', primary_key=True)  # Field name made lowercase.
    namewe = models.CharField(max_length=45, blank=True, null=True)

    class Meta:
        managed = False
        db_table = 'Weapons'


# class AuthGroup(models.Model):
#     name = models.CharField(unique=True, max_length=80)

#     class Meta:
#         managed = False
#         db_table = 'auth_group'


# class AuthGroupPermissions(models.Model):
#     group = models.ForeignKey(AuthGroup, models.DO_NOTHING)
#     permission = models.ForeignKey('AuthPermission', models.DO_NOTHING)

#     class Meta:
#         managed = False
#         db_table = 'auth_group_permissions'
#         unique_together = (('group', 'permission'),)


# class AuthPermission(models.Model):
#     name = models.CharField(max_length=255)
#     content_type = models.ForeignKey('DjangoContentType', models.DO_NOTHING)
#     codename = models.CharField(max_length=100)

#     class Meta:
#         managed = False
#         db_table = 'auth_permission'
#         unique_together = (('content_type', 'codename'),)


# class AuthUser(models.Model):
#     password = models.CharField(max_length=128)
#     last_login = models.DateTimeField(blank=True, null=True)
#     is_superuser = models.IntegerField()
#     username = models.CharField(unique=True, max_length=150)
#     first_name = models.CharField(max_length=30)
#     last_name = models.CharField(max_length=30)
#     email = models.CharField(max_length=254)
#     is_staff = models.IntegerField()
#     is_active = models.IntegerField()
#     date_joined = models.DateTimeField()

#     class Meta:
#         managed = False
#         db_table = 'auth_user'


# class AuthUserGroups(models.Model):
#     user = models.ForeignKey(AuthUser, models.DO_NOTHING)
#     group = models.ForeignKey(AuthGroup, models.DO_NOTHING)

#     class Meta:
#         managed = False
#         db_table = 'auth_user_groups'
#         unique_together = (('user', 'group'),)


# class AuthUserUserPermissions(models.Model):
#     user = models.ForeignKey(AuthUser, models.DO_NOTHING)
#     permission = models.ForeignKey(AuthPermission, models.DO_NOTHING)

#     class Meta:
#         managed = False
#         db_table = 'auth_user_user_permissions'
#         unique_together = (('user', 'permission'),)


# class DjangoAdminLog(models.Model):
#     action_time = models.DateTimeField()
#     object_id = models.TextField(blank=True, null=True)
#     object_repr = models.CharField(max_length=200)
#     action_flag = models.SmallIntegerField()
#     change_message = models.TextField()
#     content_type = models.ForeignKey('DjangoContentType', models.DO_NOTHING, blank=True, null=True)
#     user = models.ForeignKey(AuthUser, models.DO_NOTHING)

#     class Meta:
#         managed = False
#         db_table = 'django_admin_log'


# class DjangoContentType(models.Model):
#     app_label = models.CharField(max_length=100)
#     model = models.CharField(max_length=100)

#     class Meta:
#         managed = False
#         db_table = 'django_content_type'
#         unique_together = (('app_label', 'model'),)


# class DjangoMigrations(models.Model):
#     app = models.CharField(max_length=255)
#     name = models.CharField(max_length=255)
#     applied = models.DateTimeField()

#     class Meta:
#         managed = False
#         db_table = 'django_migrations'


# class DjangoSession(models.Model):
#     session_key = models.CharField(primary_key=True, max_length=40)
#     session_data = models.TextField()
#     expire_date = models.DateTimeField()

#     class Meta:
#         managed = False
#         db_table = 'django_session'
