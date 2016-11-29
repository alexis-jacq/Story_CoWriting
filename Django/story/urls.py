from django.conf.urls import url

from . import views


urlpatterns = [
    url(r'^$', views.IndexView.as_view(), name='index'),
    url(r'^$', views.Story2View, name='add_story2'),
    url(r'^$', views.FooterView, name='footer'),
    url(r'^$', views.HeaderView, name='header'),
   # url(r'^add/$', views.StoryCreate.as_view(), name='story-add'),

]


