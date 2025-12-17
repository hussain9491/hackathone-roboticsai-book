import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { motion } from 'framer-motion';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2
      }
    }
  };

  const itemVariants = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: 0.5,
        ease: [0.25, 0.46, 0.45, 0.94]
      }
    }
  };

  return (
    <motion.header
      className="hero hero--primary bg-gradient-to-br from-blue-900 to-indigo-900 text-white"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 0.8 }}
    >
      <div className="container padding-horiz--md">
        <div className="row">
          <div className="col col--6 padding-vert--lg">
            <motion.div
              variants={containerVariants}
              initial="hidden"
              animate="visible"
            >
              <motion.div variants={itemVariants}>
                <Heading as="h1" className="hero__title text-4xl md:text-5xl font-bold">
                  {siteConfig.title}
                </Heading>
              </motion.div>
              <motion.div variants={itemVariants}>
                <p className="hero__subtitle text-xl md:text-2xl mt-4 text-blue-100">
                  {siteConfig.tagline}
                </p>
              </motion.div>
              <motion.div variants={itemVariants} className="margin-top--lg">
                <div className="flex flex-col sm:flex-row gap-4">
                  <Link
                    className="button button--secondary button--lg bg-white text-blue-900 hover:bg-blue-50 transition-colors"
                    to="/docs/chapters/chapter1/section1-digital-ai-to-physical-ai">
                    Start Reading - 5min ⏱️
                  </Link>
                  <Link
                    className="button button--outline button--lg text-white border-white hover:bg-white hover:text-blue-900 transition-colors"
                    to="/docs/intro">
                    Explore Features
                  </Link>
                </div>
              </motion.div>
            </motion.div>
          </div>
          <div className="col col--6 padding-vert--lg flex items-center justify-center">
            <motion.div
              className="relative"
              initial={{ scale: 0.8, rotateY: -15 }}
              animate={{ scale: 1, rotateY: 0 }}
              transition={{ duration: 0.8, ease: "easeOut" }}
            >
              {/* 3D-style humanoid robot illustration */}
              <div className="relative w-64 h-64 mx-auto">
                {/* Robot body */}
                <motion.div
                  className="absolute w-24 h-32 bg-gradient-to-b from-gray-300 to-gray-400 rounded-lg mx-auto left-1/2 transform -translate-x-1/2"
                  animate={{
                    y: [0, -5, 0],
                  }}
                  transition={{
                    duration: 3,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut"
                  }}
                />

                {/* Robot head */}
                <motion.div
                  className="absolute w-16 h-16 bg-gradient-to-b from-gray-200 to-gray-300 rounded-full mx-auto left-1/2 transform -translate-x-1/2 -top-4"
                  animate={{
                    y: [0, -3, 0],
                  }}
                  transition={{
                    duration: 2.5,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut",
                    delay: 0.2
                  }}
                >
                  {/* Eyes */}
                  <div className="absolute top-4 left-3 w-2 h-2 bg-blue-500 rounded-full"></div>
                  <div className="absolute top-4 right-3 w-2 h-2 bg-blue-500 rounded-full"></div>
                  {/* LED indicator */}
                  <motion.div
                    className="absolute top-6 left-1/2 transform -translate-x-1/2 w-1 h-1 bg-green-400 rounded-full"
                    animate={{ scale: [1, 1.5, 1] }}
                    transition={{
                      duration: 1.5,
                      repeat: Infinity,
                      repeatType: "reverse",
                      ease: "easeInOut"
                    }}
                  />
                </motion.div>

                {/* Robot arms */}
                <motion.div
                  className="absolute w-6 h-20 bg-gradient-to-b from-gray-300 to-gray-400 rounded-lg -left-6 top-8 origin-top"
                  animate={{ rotate: [-5, 5, -5] }}
                  transition={{
                    duration: 4,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut"
                  }}
                />
                <motion.div
                  className="absolute w-6 h-20 bg-gradient-to-b from-gray-300 to-gray-400 rounded-lg -right-6 top-8 origin-top"
                  animate={{ rotate: [5, -5, 5] }}
                  transition={{
                    duration: 4,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut",
                    delay: 0.3
                  }}
                />

                {/* Robot legs */}
                <div className="absolute w-6 h-16 bg-gradient-to-b from-gray-400 to-gray-500 rounded-lg -left-2 bottom-0"></div>
                <div className="absolute w-6 h-16 bg-gradient-to-b from-gray-400 to-gray-500 rounded-lg left-10 bottom-0"></div>

                {/* Floating particles */}
                {[...Array(8)].map((_, i) => (
                  <motion.div
                    key={i}
                    className="absolute w-2 h-2 bg-blue-400 rounded-full opacity-70"
                    style={{
                      left: `${20 + (i * 10)}%`,
                      top: `${30 + (i * 5)}%`,
                    }}
                    animate={{
                      y: [0, -20, 0],
                      x: [0, Math.sin(i) * 10, 0],
                      scale: [1, 1.2, 1],
                    }}
                    transition={{
                      duration: 3 + i,
                      repeat: Infinity,
                      repeatType: "reverse",
                      ease: "easeInOut",
                      delay: i * 0.2,
                    }}
                  />
                ))}
              </div>

              {/* Floating tech elements */}
              <motion.div
                className="absolute -top-6 -right-6 text-4xl"
                animate={{
                  rotate: [0, 10, 0],
                  scale: [1, 1.1, 1]
                }}
                transition={{
                  duration: 4,
                  repeat: Infinity,
                  repeatType: "reverse",
                  ease: "easeInOut"
                }}
              >
                ⚙️
              </motion.div>
              <motion.div
                className="absolute -bottom-6 -left-6 text-4xl"
                animate={{
                  rotate: [0, -10, 0],
                  scale: [1, 1.1, 1]
                }}
                transition={{
                  duration: 3.5,
                  repeat: Infinity,
                  repeatType: "reverse",
                  ease: "easeInOut",
                  delay: 0.5
                }}
              >
                🧠
              </motion.div>
            </motion.div>
          </div>
        </div>
      </div>
    </motion.header>
  );
}

function HomepageFeatures() {
  const {siteConfig} = useDocusaurusContext();

  const features = [
    {
      title: 'Physical AI Foundation',
      description: 'Understand how AI moves from digital to physical realms, bridging the gap between algorithms and real-world interaction.',
      icon: '🤖'
    },
    {
      title: 'Humanoid Design',
      description: 'Explore the engineering challenges and solutions behind creating humanoid robots that can navigate and interact with human environments.',
      icon: '🦾'
    },
    {
      title: 'Embodied Intelligence',
      description: 'Learn how robots develop intelligence through physical interaction with the world, similar to how humans learn through experience.',
      icon: '🧠'
    }
  ];

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2
      }
    }
  };

  const itemVariants = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: 0.5,
        ease: "easeOut"
      }
    }
  };

  return (
    <section className="margin-vert--lg bg-gray-50 dark:bg-gray-800">
      <div className="container">
        <motion.div
          className="text-center margin-bottom--lg"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
        >
          <Heading as="h2" className="text-3xl font-bold text-gray-900 dark:text-white">
            What You'll Learn
          </Heading>
          <p className="text-lg text-gray-600 dark:text-gray-300 mt-2">
            Comprehensive coverage of the latest in Physical AI and Humanoid Robotics
          </p>
        </motion.div>

        <motion.div
          variants={containerVariants}
          initial="hidden"
          whileInView="visible"
          viewport={{ once: true, margin: "-100px" }}
          className="row"
        >
          {features.map((feature, index) => (
            <motion.div
              key={index}
              variants={itemVariants}
              className="col col--4 margin-bottom--lg"
            >
              <div className="text--center padding-horiz--md">
                <motion.div
                  className="text-4xl margin-bottom--sm"
                  whileHover={{ scale: 1.2, rotate: 5 }}
                  transition={{ type: "spring", stiffness: 300 }}
                >
                  {feature.icon}
                </motion.div>
                <Heading as="h3" className="text-xl font-semibold text-gray-900 dark:text-white">
                  {feature.title}
                </Heading>
                <p className="text-gray-600 dark:text-gray-300">
                  {feature.description}
                </p>
              </div>
            </motion.div>
          ))}
        </motion.div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An interactive textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
