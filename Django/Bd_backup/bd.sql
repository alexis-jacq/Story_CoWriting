-- MySQL dump 10.13  Distrib 5.7.16, for Linux (x86_64)
--
-- Host: localhost    Database: storytelling
-- ------------------------------------------------------
-- Server version	5.7.16-0ubuntu0.16.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `Accessories`
--

DROP TABLE IF EXISTS `Accessories`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Accessories` (
  `idAccessories` int(11) NOT NULL,
  `nameAc` varchar(45) DEFAULT NULL,
  `idDr` int(11) NOT NULL,
  `idFo` int(11) NOT NULL,
  `idTo` int(11) NOT NULL,
  PRIMARY KEY (`idAccessories`),
  KEY `fk_Accessories_1_idx` (`idDr`),
  KEY `fk_Accessories_2_idx` (`idTo`),
  KEY `fk_Accessories_3_idx` (`idFo`),
  CONSTRAINT `fk_Accessories_1` FOREIGN KEY (`idDr`) REFERENCES `Drinks` (`idDrinks`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Accessories_2` FOREIGN KEY (`idTo`) REFERENCES `Tools` (`idTools`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Accessories_3` FOREIGN KEY (`idFo`) REFERENCES `Foods` (`idFoods`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Accessories`
--

LOCK TABLES `Accessories` WRITE;
/*!40000 ALTER TABLE `Accessories` DISABLE KEYS */;
/*!40000 ALTER TABLE `Accessories` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Categories`
--

DROP TABLE IF EXISTS `Categories`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Categories` (
  `idCategories` int(11) NOT NULL,
  `nameCa` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idCategories`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Categories`
--

LOCK TABLES `Categories` WRITE;
/*!40000 ALTER TABLE `Categories` DISABLE KEYS */;
/*!40000 ALTER TABLE `Categories` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Ch_Ac`
--

DROP TABLE IF EXISTS `Ch_Ac`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Ch_Ac` (
  `idCh_Ac` int(11) NOT NULL,
  `idCh` int(11) NOT NULL,
  `idAc` int(11) NOT NULL,
  PRIMARY KEY (`idCh_Ac`),
  KEY `fk_Ch_Ac_1_idx` (`idCh`),
  KEY `fk_Ch_Ac_2_idx` (`idAc`),
  CONSTRAINT `fk_Ch_Ac_1` FOREIGN KEY (`idCh`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Ch_Ac_2` FOREIGN KEY (`idAc`) REFERENCES `Accessories` (`idAccessories`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Ch_Ac`
--

LOCK TABLES `Ch_Ac` WRITE;
/*!40000 ALTER TABLE `Ch_Ac` DISABLE KEYS */;
/*!40000 ALTER TABLE `Ch_Ac` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Characters`
--

DROP TABLE IF EXISTS `Characters`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Characters` (
  `idCharacters` int(11) NOT NULL AUTO_INCREMENT,
  `nameCh` varchar(45) DEFAULT NULL,
  `gender` varchar(1) DEFAULT NULL COMMENT 'M - Male\nF - Female',
  `idRo` int(11) NOT NULL,
  `idJo` int(11) NOT NULL,
  `idSp` int(11) NOT NULL,
  PRIMARY KEY (`idCharacters`),
  KEY `fk_Characters_1_idx` (`idRo`),
  KEY `fk_Characters_2_idx` (`idJo`),
  KEY `fk_Characters_3_idx` (`idSp`),
  CONSTRAINT `fk_Characters_1` FOREIGN KEY (`idRo`) REFERENCES `Role` (`idRole`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Characters_2` FOREIGN KEY (`idJo`) REFERENCES `Job` (`idJobs`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Characters_3` FOREIGN KEY (`idSp`) REFERENCES `Species` (`idSpecies`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Characters`
--

LOCK TABLES `Characters` WRITE;
/*!40000 ALTER TABLE `Characters` DISABLE KEYS */;
/*!40000 ALTER TABLE `Characters` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Containers`
--

DROP TABLE IF EXISTS `Containers`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Containers` (
  `idContainers` int(11) NOT NULL,
  `nameCo` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idContainers`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Containers`
--

LOCK TABLES `Containers` WRITE;
/*!40000 ALTER TABLE `Containers` DISABLE KEYS */;
/*!40000 ALTER TABLE `Containers` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Drinks`
--

DROP TABLE IF EXISTS `Drinks`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Drinks` (
  `idDrinks` int(11) NOT NULL,
  `nameDr` varchar(45) DEFAULT NULL,
  `idCo` int(11) NOT NULL,
  PRIMARY KEY (`idDrinks`),
  KEY `fk_Drinks_1_idx` (`idCo`),
  CONSTRAINT `fk_Drinks_1` FOREIGN KEY (`idCo`) REFERENCES `Containers` (`idContainers`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Drinks`
--

LOCK TABLES `Drinks` WRITE;
/*!40000 ALTER TABLE `Drinks` DISABLE KEYS */;
/*!40000 ALTER TABLE `Drinks` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Enemies`
--

DROP TABLE IF EXISTS `Enemies`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Enemies` (
  `idEnemies` int(11) NOT NULL,
  `idCh` int(11) NOT NULL,
  `idCh_` int(11) NOT NULL,
  PRIMARY KEY (`idEnemies`),
  KEY `fk_Enemies_1_idx` (`idCh`),
  KEY `fk_Enemies_2_idx` (`idCh_`),
  CONSTRAINT `fk_Enemies_1` FOREIGN KEY (`idCh`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Enemies_2` FOREIGN KEY (`idCh_`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Enemies`
--

LOCK TABLES `Enemies` WRITE;
/*!40000 ALTER TABLE `Enemies` DISABLE KEYS */;
/*!40000 ALTER TABLE `Enemies` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Foods`
--

DROP TABLE IF EXISTS `Foods`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Foods` (
  `idFoods` int(11) NOT NULL,
  `nameFo` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idFoods`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Foods`
--

LOCK TABLES `Foods` WRITE;
/*!40000 ALTER TABLE `Foods` DISABLE KEYS */;
/*!40000 ALTER TABLE `Foods` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Friends`
--

DROP TABLE IF EXISTS `Friends`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Friends` (
  `idFriends` int(11) NOT NULL,
  `idCh` int(11) NOT NULL,
  `idCh_` int(11) NOT NULL,
  PRIMARY KEY (`idFriends`),
  KEY `fk_Friends_1_idx` (`idCh`),
  KEY `fk_Friends_2_idx` (`idCh_`),
  CONSTRAINT `fk_Friends_1` FOREIGN KEY (`idCh`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Friends_2` FOREIGN KEY (`idCh_`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Friends`
--

LOCK TABLES `Friends` WRITE;
/*!40000 ALTER TABLE `Friends` DISABLE KEYS */;
/*!40000 ALTER TABLE `Friends` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Job`
--

DROP TABLE IF EXISTS `Job`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Job` (
  `idJobs` int(11) NOT NULL,
  `nameJo` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idJobs`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Job`
--

LOCK TABLES `Job` WRITE;
/*!40000 ALTER TABLE `Job` DISABLE KEYS */;
/*!40000 ALTER TABLE `Job` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Role`
--

DROP TABLE IF EXISTS `Role`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Role` (
  `idRole` int(11) NOT NULL,
  `nameRo` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idRole`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Role`
--

LOCK TABLES `Role` WRITE;
/*!40000 ALTER TABLE `Role` DISABLE KEYS */;
/*!40000 ALTER TABLE `Role` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Scenarios`
--

DROP TABLE IF EXISTS `Scenarios`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Scenarios` (
  `idScenarios` int(11) NOT NULL,
  PRIMARY KEY (`idScenarios`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Scenarios`
--

LOCK TABLES `Scenarios` WRITE;
/*!40000 ALTER TABLE `Scenarios` DISABLE KEYS */;
/*!40000 ALTER TABLE `Scenarios` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Species`
--

DROP TABLE IF EXISTS `Species`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Species` (
  `idSpecies` int(11) NOT NULL,
  `nameSp` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`idSpecies`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Species`
--

LOCK TABLES `Species` WRITE;
/*!40000 ALTER TABLE `Species` DISABLE KEYS */;
/*!40000 ALTER TABLE `Species` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `St_Ch`
--

DROP TABLE IF EXISTS `St_Ch`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `St_Ch` (
  `idSt_Ch` int(11) NOT NULL,
  `idSt` int(11) NOT NULL,
  `idCh` int(11) NOT NULL,
  PRIMARY KEY (`idSt_Ch`),
  KEY `fk_St_Ch_1_idx` (`idCh`),
  KEY `fk_St_Ch_2_idx` (`idSt`),
  CONSTRAINT `fk_St_Ch_1` FOREIGN KEY (`idCh`) REFERENCES `Characters` (`idCharacters`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_St_Ch_2` FOREIGN KEY (`idSt`) REFERENCES `Story` (`idStory`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `St_Ch`
--

LOCK TABLES `St_Ch` WRITE;
/*!40000 ALTER TABLE `St_Ch` DISABLE KEYS */;
/*!40000 ALTER TABLE `St_Ch` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Story`
--

DROP TABLE IF EXISTS `Story`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Story` (
  `idStory` int(11) NOT NULL,
  `nameSt` varchar(45) DEFAULT NULL,
  `script` varchar(45) DEFAULT NULL,
  `idTy` int(11) NOT NULL,
  `idSc` int(11) NOT NULL,
  PRIMARY KEY (`idStory`),
  KEY `fk_Story_1_idx` (`idTy`),
  KEY `fk_Story_2_idx` (`idSc`),
  CONSTRAINT `fk_Story_1` FOREIGN KEY (`idTy`) REFERENCES `Type` (`idType`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Story_2` FOREIGN KEY (`idSc`) REFERENCES `Scenarios` (`idScenarios`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Story`
--

LOCK TABLES `Story` WRITE;
/*!40000 ALTER TABLE `Story` DISABLE KEYS */;
/*!40000 ALTER TABLE `Story` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Tools`
--

DROP TABLE IF EXISTS `Tools`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Tools` (
  `idTools` int(11) NOT NULL,
  `nameTo` varchar(45) DEFAULT NULL,
  `idCa` int(11) NOT NULL,
  `idJo` int(11) NOT NULL,
  PRIMARY KEY (`idTools`),
  KEY `fk_Tools_1_idx` (`idCa`),
  KEY `fk_Tools_2_idx` (`idJo`),
  CONSTRAINT `fk_Tools_1` FOREIGN KEY (`idCa`) REFERENCES `Categories` (`idCategories`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_Tools_2` FOREIGN KEY (`idJo`) REFERENCES `Job` (`idJobs`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Tools`
--

LOCK TABLES `Tools` WRITE;
/*!40000 ALTER TABLE `Tools` DISABLE KEYS */;
/*!40000 ALTER TABLE `Tools` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Type`
--

DROP TABLE IF EXISTS `Type`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `Type` (
  `idType` int(11) NOT NULL,
  PRIMARY KEY (`idType`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Type`
--

LOCK TABLES `Type` WRITE;
/*!40000 ALTER TABLE `Type` DISABLE KEYS */;
/*!40000 ALTER TABLE `Type` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2016-11-16 15:17:38
