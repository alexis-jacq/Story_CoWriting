"""StorytellingGenerator URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.10/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from django.conf.urls import url, include
from django.contrib import admin
from rest_framework.urlpatterns import format_suffix_patterns
from story import views

urlpatterns = [
	url(r'^$', include('story.urls')),
 	url(r'^story/', views.StoryView.as_view(),name='add_story'),
 	url(r'^story2/', views.StoryList.as_view(),name='add_story2'),
 	url(r'^st/', views.FullStoryList.as_view(),name='story'),
	url(r'^st/teste', views.FullStoryList.storyteste, name='storyteste'),

    url(r'^admin/', admin.site.urls),
]

urlpatterns = format_suffix_patterns(urlpatterns)